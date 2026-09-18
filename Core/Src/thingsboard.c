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

static bool ThingsBoard_SendTelemetry(const char *json)
{
    /* Retry connection setup after an earlier failure, but only when called for
     * a fall or its acknowledgement; no periodic retries in the sensor loop. */
    if (!network_ready && !ThingsBoard_Init()) return false;

    char request[768];
    size_t json_length = strlen(json);

    /* Content-Length counts only the JSON body. Request a fresh connection for
     * each update because the server may close an idle socket between events. */
    int request_length = snprintf(request, sizeof(request),
        "POST /api/v1/%s/telemetry HTTP/1.1\r\n"
        "Host: %s:%u\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n\r\n%s",
        THINGSBOARD_ACCESS_TOKEN, THINGSBOARD_HOST,
        (unsigned int)THINGSBOARD_PORT, (unsigned int)json_length, json);
    if (request_length < 0 || (size_t)request_length >= sizeof(request)) return false;

    if (WIFI_OpenClientConnection(THINGSBOARD_SOCKET, WIFI_TCP_PROTOCOL,
            "fall", server_ip, THINGSBOARD_PORT, THINGSBOARD_LOCAL_PORT)
            != WIFI_STATUS_OK) {
        network_ready = false;
        return false;
    }

    bool delivered = false;
    uint16_t sent = 0;
    if (WIFI_SendData(THINGSBOARD_SOCKET, (uint8_t *)request,
            (uint16_t)request_length, &sent, THINGSBOARD_IO_TIMEOUT_MS)
            == WIFI_STATUS_OK && sent == (uint16_t)request_length) {
        /* TCP may split the HTTP status line across reads. Accumulate until its
         * CRLF, leaving one byte for '\0'; inspect the actual status, not a
         * substring "200" that could also appear elsewhere in the response. */
        char response[256] = {0};
        size_t used = 0;
        uint32_t started_at = HAL_GetTick();
        while (used < sizeof(response) - 1U) {
            uint32_t elapsed = HAL_GetTick() - started_at;
            if (elapsed >= THINGSBOARD_IO_TIMEOUT_MS) break;
            uint16_t received = 0;
            uint16_t available = (uint16_t)(sizeof(response) - 1U - used);
            WIFI_Status_t status = WIFI_ReceiveData(THINGSBOARD_SOCKET,
                (uint8_t *)response + used, available, &received,
                THINGSBOARD_IO_TIMEOUT_MS - elapsed);
            if (status != WIFI_STATUS_OK || received == 0 || received > available) break;
            used += received;
            response[used] = '\0';
            if (strstr(response, "\r\n") != NULL) {
                unsigned int http_status = 0;
                delivered = sscanf(response, "HTTP/%*u.%*u %u", &http_status) == 1
                            && http_status == 200U;
                break;
            }
        }
    }

    /* Closing also releases the module socket. A lost response is treated as
     * unconfirmed delivery, even if the server actually accepted the event. */
    WIFI_Status_t close_status = WIFI_CloseClientConnection(THINGSBOARD_SOCKET);
    if (!delivered || close_status != WIFI_STATUS_OK) network_ready = false;
    return delivered;
}
