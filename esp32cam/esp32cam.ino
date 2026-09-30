/*
 * esp32cam.ino - ESP32-CAM firmware: video stream + phone web control.
 *
 * Three jobs:
 *  1. Serve MJPEG video on   http://192.168.4.1:81/stream
 *  2. Serve phone UI on      http://192.168.4.1/   (joystick + gimbal sliders)
 *  3. Bridge phone commands  -> UART (U0TXD GPIO1 -> rover Uno pin 0)
 *     and relay rover telemetry (U0RXD GPIO3 <- rover Uno pin 1) to the UI.
 *
 * Phone commands are sent to the rover as standard protocol frames
 * (CMD_DRIVE + CMD_GIMBAL). The rover arbitrates with the PC radio link.
 *
 * NOTE: index.html in this folder is a readable copy of the page embedded
 * below. Edit ONE, mirror to the OTHER.
 *
 * Flash with PSRAM enabled:
 *   arduino-cli compile -b esp32:esp32:esp32:PSRAM=enabled esp32cam
 */
#include "esp_camera.h"
#include <WiFi.h>
#include <esp_http_server.h>
#include "protocol.h"

/* ---------------- WiFi (AP mode, no router needed) ---------------- */
const char *AP_SSID = "RoverCam";
const char *AP_PASS = "rover1234";

/* ---------------- camera (AI Thinker ESP32-CAM pinout) ---------------- */
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

#define XCLK_FREQ_HZ   20000000
#define FRAMESIZE      FRAMESIZE_QVGA   /* 320x240: smooth over WiFi */
#define JPEG_QUALITY   10

/* ---------------- shared state ---------------- */
proto_parser_t telemParser;
uint8_t rxBuf[PROTO_FRAME_MAX];
uint8_t txBuf[PROTO_FRAME_MAX];

volatile uint16_t telemDist = 0;
volatile uint16_t telemBatt = 0;
volatile uint8_t  telemLoop = 0;
volatile uint8_t  telemRadio = 0;

volatile int16_t cmdThrottle = 0;
volatile int16_t cmdSteer = 0;
volatile uint8_t cmdPan = 90;
volatile uint8_t cmdTilt = 90;
volatile bool    cmdDirty = false;

httpd_handle_t uiServer = NULL;
httpd_handle_t streamServer = NULL;

/* ---------------- embedded phone UI (mirror of index.html) ---------------- */
static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<!--
  Rover phone control UI. Served by esp32cam.ino at http://192.168.4.1/
  This file must stay byte-identical to the INDEX_HTML string embedded in
  esp32cam.ino (tested by tests/test_protocol.py).
-->
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Rover</title>
<style>
html,body{margin:0;height:100%;overflow:hidden;background:#111;color:#ddd;font-family:system-ui,sans-serif}
#video{position:fixed;inset:0;width:100%;height:100%;object-fit:cover}
.tele{position:fixed;top:8px;left:8px;display:flex;gap:8px;z-index:5}
.chip{background:rgba(0,0,0,.6);padding:4px 8px;border-radius:6px;font-size:13px}
#joy{position:fixed;left:12px;bottom:12px;width:150px;height:150px;border-radius:50%;
 background:rgba(255,255,255,.12);border:2px solid rgba(255,255,255,.4);z-index:5;touch-action:none}
#knob{position:absolute;left:62px;top:62px;width:26px;height:26px;border-radius:50%;
 background:rgba(255,255,255,.85);pointer-events:none}
.sliders{position:fixed;right:12px;bottom:12px;z-index:5;display:flex;flex-direction:column;gap:14px}
.slider{display:flex;flex-direction:column;align-items:center;font-size:11px;gap:2px}
input[type=range]{writing-mode:vertical-lr;direction:rtl;width:34px;height:140px}
</style>
</head><body>
<img id="video" src="/stream">
<div class="tele">
  <div class="chip" id="cDist">-- cm</div>
  <div class="chip" id="cBatt">-- V</div>
  <div class="chip" id="cRadio">--</div>
  <div class="chip" id="cLoop">-- ms</div>
</div>
<div id="joy"><div id="knob"></div></div>
<div class="sliders">
  <div class="slider">PAN<input id="pan" type="range" min="0" max="180" value="90"></div>
  <div class="slider">TILT<input id="tilt" type="range" min="0" max="180" value="90"></div>
</div>
<script>
var joy=document.getElementById('joy'),knob=document.getElementById('knob');
var jx=0,jy=0,active=false,R=58,lastQ=null;
var pan=document.getElementById('pan'),tilt=document.getElementById('tilt');
function setKnob(){knob.style.left=(62+jx-13)+'px';knob.style.top=(62+jy-13)+'px';}
function update(x,y){
  var r=joy.getBoundingClientRect();
  var dx=x-(r.left+r.width/2),dy=y-(r.top+r.height/2);
  var len=Math.sqrt(dx*dx+dy*dy);
  if(len>R){dx=dx/len*R;dy=dy/len*R;}
  jx=dx;jy=dy;setKnob();send();
}
function send(){
  var throttle=-Math.round(jy/R*255);
  var steer=Math.round(jx/R*255);
  var q=[throttle,steer,pan.value,tilt.value].join(',');
  if(!active&&q===lastQ)return;
  lastQ=q;
  fetch('/cmd?t='+throttle+'&s='+steer+'&p='+pan.value+'&tl='+tilt.value);
}
joy.addEventListener('touchstart',function(e){update(e.touches[0].clientX,e.touches[0].clientY);active=true;e.preventDefault();},{passive:false});
joy.addEventListener('touchmove',function(e){update(e.touches[0].clientX,e.touches[0].clientY);e.preventDefault();},{passive:false});
joy.addEventListener('touchend',function(){active=false;jx=jy=0;setKnob();send();});
joy.addEventListener('mousedown',function(e){update(e.clientX,e.clientY);active=true;e.preventDefault();});
window.addEventListener('mousemove',function(e){if(active)update(e.clientX,e.clientY);});
window.addEventListener('mouseup',function(){if(active){active=false;jx=jy=0;setKnob();send();}});
pan.addEventListener('input',send);tilt.addEventListener('input',send);
setInterval(function(){
  fetch('/telemetry').then(function(r){return r.json()}).then(function(t){
    document.getElementById('cDist').textContent=t.dist+' cm';
    document.getElementById('cBatt').textContent=(t.batt/1000).toFixed(2)+' V';
    document.getElementById('cRadio').textContent=t.radio?'RADIO OK':'NO RADIO';
    document.getElementById('cLoop').textContent=t.loop+' ms';
  }).catch(function(){});
},500);
</script>
</body></html>
)rawliteral";

/* ---------------- camera init ---------------- */
void initCamera() {
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
  config.xclk_freq_hz = XCLK_FREQ_HZ;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s) s->set_framesize(s, FRAMESIZE);
}

/* ---------------- HTTP handlers ---------------- */
static const char *STREAM_BOUNDARY = "--frameboundary\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=--frameboundary";

static esp_err_t handleIndex(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t handleCmd(httpd_req_t *req) {
  char buf[64];
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) != ESP_OK) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  char tmp[16];
  int t = 0, s = 0, p = 90, tl = 90;
  if (httpd_query_key_value(buf, "t", tmp, sizeof(tmp)) == ESP_OK) t = atoi(tmp);
  if (httpd_query_key_value(buf, "s", tmp, sizeof(tmp)) == ESP_OK) s = atoi(tmp);
  if (httpd_query_key_value(buf, "p", tmp, sizeof(tmp)) == ESP_OK) p = atoi(tmp);
  if (httpd_query_key_value(buf, "tl", tmp, sizeof(tmp)) == ESP_OK) tl = atoi(tmp);
  cmdThrottle = constrain(t, -255, 255);
  cmdSteer = constrain(s, -255, 255);
  cmdPan = (uint8_t)constrain(p, 0, 180);
  cmdTilt = (uint8_t)constrain(tl, 0, 180);
  cmdDirty = true;
  return httpd_resp_send(req, "ok", 2);
}

static esp_err_t handleTelemetry(httpd_req_t *req) {
  char json[80];
  int n = snprintf(json, sizeof(json),
                   "{\"dist\":%u,\"batt\":%u,\"radio\":%u,\"loop\":%u}",
                   telemDist, telemBatt, telemRadio, telemLoop);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, json, n);
}

static esp_err_t handleStream(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  char part[64];
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
      break;
    }
    int hlen = snprintf(part, sizeof(part), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, part, (ssize_t)hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, (ssize_t)fb->len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, (ssize_t)strlen(STREAM_BOUNDARY));
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }
  return res;
}

void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.lru_purge_enable = true;

  cfg.server_port = 80;
  httpd_start(&uiServer, &cfg);
  httpd_uri_t uiIdx = {.uri = "/", .method = HTTP_GET, .handler = handleIndex, .user_ctx = NULL};
  httpd_uri_t uiCmd = {.uri = "/cmd", .method = HTTP_GET, .handler = handleCmd, .user_ctx = NULL};
  httpd_uri_t uiTel = {.uri = "/telemetry", .method = HTTP_GET, .handler = handleTelemetry, .user_ctx = NULL};
  httpd_register_uri_handler(uiServer, &uiIdx);
  httpd_register_uri_handler(uiServer, &uiCmd);
  httpd_register_uri_handler(uiServer, &uiTel);

  cfg.server_port = 81;
  httpd_start(&streamServer, &cfg);
  httpd_uri_t stream = {.uri = "/stream", .method = HTTP_GET, .handler = handleStream, .user_ctx = NULL};
  httpd_register_uri_handler(streamServer, &stream);
}

/* ---------------- setup / loop ---------------- */
void setup() {
  Serial.begin(115200);
  proto_parser_reset(&telemParser);

  initCamera();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);

  startServers();
}

unsigned long lastSendMs = 0;

void loop() {
  /* rover telemetry -> web page state */
  while (Serial.available()) {
    if (proto_feed(&telemParser, (uint8_t)Serial.read())) {
      proto_frame_t *f = &telemParser.out;
      if (f->type == PROTO_TELEM) {
        telemDist = proto_get_u16(f->payload);
        telemBatt = proto_get_u16(f->payload + 2);
        telemLoop = f->payload[4];
        telemRadio = f->payload[5];
      } else if (f->type == PROTO_PING) {
        proto_frame_t ack = {PROTO_ACK, 0, {0}};
        size_t n = proto_encode(&ack, txBuf, sizeof(txBuf));
        Serial.write(txBuf, n);
      }
    }
  }

  /* phone commands -> rover, 20 Hz */
  if (cmdDirty && millis() - lastSendMs >= 50) {
    lastSendMs = millis();
    cmdDirty = false;

    proto_frame_t f;
    f.type = PROTO_CMD_DRIVE;
    f.len = 4;
    proto_put_i16(f.payload, cmdThrottle);
    proto_put_i16(f.payload + 2, cmdSteer);
    size_t n = proto_encode(&f, txBuf, sizeof(txBuf));
    Serial.write(txBuf, n);

    f.type = PROTO_CMD_GIMBAL;
    f.len = 2;
    f.payload[0] = cmdPan;
    f.payload[1] = cmdTilt;
    n = proto_encode(&f, txBuf, sizeof(txBuf));
    Serial.write(txBuf, n);
  }

  delay(5);
}
