#include "thingsboard.h"
#include "wifi.h"
#include <stdio.h>
#include <string.h>

static bool network_ready;
static uint8_t server_ip[4];
static bool ThingsBoard_SendTelemetry(const char *json);

bool ThingsBoard_Init(void)
{
    network_ready = false;
    if (WIFI_SSID[0] == '\0' || THINGSBOARD_ACCESS_TOKEN[0] == '\0') {
        return false;
    }

    /* WIFI_STATUS_OK is zero: combining results with &= hides failures. */
    if (WIFI_Init() != WIFI_STATUS_OK) return false;
    if (WIFI_Connect(WIFI_SSID, WIFI_PASSWORD, WIFI_SECURITY) != WIFI_STATUS_OK) {
        return false;
    }
    if (WIFI_GetHostAddress(THINGSBOARD_HOST, server_ip) != WIFI_STATUS_OK) {
        return false;
    }
    network_ready = true;
    return true;
}

bool ThingsBoard_SendFall(float acceleration_g, float angular_velocity_dps,
                         uint32_t detected_at_ms)
{
    char json[192];
    int json_length = snprintf(json, sizeof(json),
        "{\"state\":\"FALL_CONFIRMED\",\"acceleration_g\":%.3f,"
        "\"angular_velocity_dps\":%.3f,\"detected_at_uptime_ms\":%lu}",
        (double)acceleration_g, (double)angular_velocity_dps,
        (unsigned long)detected_at_ms);
    if (json_length < 0 || (size_t)json_length >= sizeof(json)) return false;
    return ThingsBoard_SendTelemetry(json);
}

bool ThingsBoard_SendNormal(void)
{
    /* Update the current state while retaining the previous fall's history and
     * sensor snapshot. The ThingsBoard alarm rule clears when state is NORMAL. */
    return ThingsBoard_SendTelemetry("{\"state\":\"NORMAL\"}");
}

bool ThingsBoard_SendLongLie(void)
{
    /* Escalate the state while retaining the fall's sensor snapshot and time. */
    return ThingsBoard_SendTelemetry("{\"state\":\"LONG_LIE\"}");
}

/* Send one JSON telemetry update.
 * Returns true only when an HTTP 200 response is received.
 * This function blocks while connecting, sending and receiving. */
static bool ThingsBoard_SendTelemetry(const char *json)
{
    /* If setup previously failed, retry it now. Short-circuit evaluation
     * skips ThingsBoard_Init() when network_ready is already true.
     * Retries happen only on event updates, not periodically during sampling. */
    if (!network_ready && !ThingsBoard_Init()) return false;

    char request[768];
    size_t json_length = strlen(json); /* Body length in bytes, excluding '\0'. */

    /* Build the HTTP request:
     * - The access token identifies/authenticates the ThingsBoard device.
     * - Host identifies the destination server.
     * - Content-Type tells the server that the body contains JSON.
     * - Content-Length counts only the body, excluding headers and '\0'.
     * - The blank line (\r\n\r\n) separates headers from the JSON body.
     * - Connection: close asks the server to close after its response;
     *   each event uses a fresh connection. */
    int request_length = snprintf(request, sizeof(request),
        "POST /api/v1/%s/telemetry HTTP/1.1\r\n"
        "Host: %s:%u\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n\r\n%s",
        THINGSBOARD_ACCESS_TOKEN, THINGSBOARD_HOST,
        (unsigned int)THINGSBOARD_PORT, (unsigned int)json_length, json);

    /* snprintf returns the required length, excluding '\0'.
     * A negative result means formatting failed; a result at least as large
     * as the buffer means truncation. Do not send an incomplete request. */
    if (request_length < 0 || (size_t)request_length >= sizeof(request))
        return false;

    /* Open the module's TCP socket to the previously resolved server IP.
     * This establishes the transport connection; the HTTP request is sent next. */
    if (WIFI_OpenClientConnection(THINGSBOARD_SOCKET, WIFI_TCP_PROTOCOL,
            "fall", server_ip, THINGSBOARD_PORT, THINGSBOARD_LOCAL_PORT)
            != WIFI_STATUS_OK) {
        /* Force setup to be retried on the next event update. */
        network_ready = false;
        return false;
    }

    bool delivered = false;
    uint16_t sent = 0;

    /* Send the request without its terminating '\0'.
     * Require both a successful driver status and the full byte count.
     * Sending successfully alone does not prove ThingsBoard accepted it. */
    if (WIFI_SendData(THINGSBOARD_SOCKET, (uint8_t *)request,
            (uint16_t)request_length, &sent, THINGSBOARD_IO_TIMEOUT_MS)
            == WIFI_STATUS_OK && sent == (uint16_t)request_length) {

        /* TCP is a byte stream: one receive call may return only part of
         * the HTTP status line. Append successive chunks until its CRLF
         * arrives, the buffer fills, or the receive timeout expires. */
        char response[256] = {0};
        size_t used = 0;
        uint32_t started_at = HAL_GetTick();

        while (used < sizeof(response) - 1U) {
            /* Use one overall timeout budget for this receive phase,
             * rather than restarting a full timeout for every chunk. */
            uint32_t elapsed = HAL_GetTick() - started_at;
            if (elapsed >= THINGSBOARD_IO_TIMEOUT_MS) break;

            uint16_t received = 0;

            /* Reserve one byte for '\0' so string functions can safely
             * inspect the accumulated response. */
            uint16_t available = (uint16_t)(sizeof(response) - 1U - used);

            /* Append at the first unused byte, using the remaining time. */
            WIFI_Status_t status = WIFI_ReceiveData(THINGSBOARD_SOCKET,
                (uint8_t *)response + used, available, &received,
                THINGSBOARD_IO_TIMEOUT_MS - elapsed);

            /* Stop on a receive error, no data, or an invalid byte count. */
            if (status != WIFI_STATUS_OK ||
                received == 0 || received > available) break;

            used += received;
            response[used] = '\0';

            /* The first CRLF terminates the HTTP status line,
             * for example: "HTTP/1.1 200 OK\r\n". */
            if (strstr(response, "\r\n") != NULL) {
                unsigned int http_status = 0;

                /* %*u consumes each HTTP version number without storing it.
                 * %u stores the status code; sscanf must make one assignment.
                 * Check the status itself, not an arbitrary "200" substring.
                 * Headers and response body are not needed for this check. */
                delivered =
                    sscanf(response, "HTTP/%*u.%*u %u", &http_status) == 1
                    && http_status == 200U;
                break;
            }
        }
    }

    /* Release the socket after every successful open, including when
     * sending failed or no valid HTTP 200 response was received.
     * Missing confirmation does not prove the server rejected the update:
     * it may have accepted it but its response was lost. */
    WIFI_Status_t close_status =
        WIFI_CloseClientConnection(THINGSBOARD_SOCKET);

    /* Retry network setup on the next event if delivery was unconfirmed
     * or socket cleanup failed. A cleanup failure does not undo an
     * already received HTTP 200, so delivered can still remain true. */
    if (!delivered || close_status != WIFI_STATUS_OK) network_ready = false;

    return delivered;
}
