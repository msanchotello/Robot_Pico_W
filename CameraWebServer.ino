#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

const char* ssid = "Galaxy A53 5G 01C3";
const char* password = "Miguel123";

WebServer server(80);

void handleRoot() {
  // Redirigir a la página de transmisión
  server.sendHeader("Location", "/stream");
  server.send(302);  // Código de redirección HTTP
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

void handleJpgStream() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "multipart/x-mixed-replace; boundary=frame");

  while (true) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      server.send(500, "text/plain", "Camera capture failed");
      return;
    }

    server.sendContent("--frame\r\n");
    server.sendContent("Content-Type: image/jpeg\r\n");
    server.sendContent("Content-Length: " + String(fb->len) + "\r\n\r\n");
    server.sendContent((const char*)fb->buf, fb->len);
    server.sendContent("\r\n");
    esp_camera_fb_return(fb);

    if (server.client().connected()) {
      delay(100);  // Ajusta este valor según sea necesario para controlar la velocidad de fotogramas
    } else {
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;    // Aumenta la resolución a 640x480
    config.jpeg_quality = 20;             // Mantiene una calidad moderada de JPEG
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;   // Respaldo con resolución 320x240 si no hay PSRAM
    config.jpeg_quality = 25;             // Calidad ligeramente inferior para mantener rendimiento
    config.fb_count = 1;
  }



  // Inicialización de la cámara
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  server.on("/", handleRoot);
  server.on("/stream", HTTP_GET, handleJpgStream);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.printf("Camera Ready! Use 'http://%s/stream' to connect\n", WiFi.localIP().toString().c_str());
}

void loop() {
  server.handleClient();
}
