#ifndef THINGSBOARD_H
#define THINGSBOARD_H

#include <stdbool.h>
#include <stdint.h>

/* Fill these in before flashing. Empty SSID/token skips network operations.
 * Use the phone hotspot's credentials, not the home desktop's Wi-Fi details.
 * Do not commit real passwords or access tokens to a public repository. */
#define WIFI_SSID                   "utakata"
#define WIFI_PASSWORD               "phrolova"
#define THINGSBOARD_ACCESS_TOKEN    "hifratol541vfu6wfmi6" // "804rcl3a1oxmcx3ock9k"
#define WIFI_SECURITY               WIFI_ECN_WPA2_PSK

/* Hostname only: no http:// prefix or path. Match the server hosting your device.
 * This follows the supplied plain-HTTP sample. Port 443 alone cannot enable TLS. */
#define THINGSBOARD_HOST            "thingsboard.cloud"
#define THINGSBOARD_PORT            80U
#define THINGSBOARD_SOCKET          1U     /* Module connection slot, not a port. */
#define THINGSBOARD_LOCAL_PORT      1234U
#define THINGSBOARD_IO_TIMEOUT_MS   5000U  /* Server send/receive wait, in ms.
                                           * Driver commands separately use 30000 ms. */

/* Blocking calls: initialise before sampling; upload on fall/acknowledgement.
 * False means unavailable/not confirmed delivered; it never stops the program. */
bool ThingsBoard_Init(void);
bool ThingsBoard_SendFall(float acceleration_g, float angular_velocity_dps,
                         uint32_t detected_at_ms);
bool ThingsBoard_SendNormal(void);

#endif
