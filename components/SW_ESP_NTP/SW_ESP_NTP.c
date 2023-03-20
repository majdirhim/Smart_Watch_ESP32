

#include "SW_ESP_NTP.h"


void NTP_Init()
{

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    //sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();

}

void NTP_GetTime(void)
{
    NTP_Init();
    // Attendre que la synchronisation NTP soit effectuée
    int retry = 0;
    const int retry_max = 5;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_max) {
        printf("Waiting for NTP server connection...\n HELLOOOO");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    // Si la synchronisation a échoué, récupérer l'heure système
    time_t now;
    struct tm timeinfo, *temps;
    if (retry == retry_max) {
        printf("NTP server non atteingnable. on utilise le local time.");//Le serveur n'est pas joignable
        time(&now);
        localtime_r(&now, &timeinfo);
    } else {
        time(&now);
        setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
        tzset();
        localtime_r(&now, &timeinfo);
    }

    // Afficher l'heure pour VERIFIFCATION!!!!
    printf("Le temps est heure %d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);


}

//TIME TM DEJA INSTALLER DANS ESPRESSIF!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void time_sync_notification_cb(struct timeval *tv)
{


}


