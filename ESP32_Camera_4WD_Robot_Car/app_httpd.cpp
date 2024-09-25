#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"

#define LEFT_M0     13
#define LEFT_M1     12
#define RIGHT_M0    14
#define RIGHT_M1    15
int speed = 150;
int noStop = 0;
const int freq = 2000;
const int motorPWMChannnel = 8;
const int lresolution = 8;
volatile unsigned int motor_speed = 100;
void robot_setup();
void robot_stop();
void robot_fwd();
void robot_back();
void robot_left();
void robot_right();
volatile unsigned long previous_time = 0;
volatile unsigned long move_interval = 250;
bool live_feed_enabled = true; // Toggle for live feed on/off

unsigned int get_speed(unsigned int sp)
{
  return map(sp, 0, 100, 0, 255);
}

void robot_setup()
{
  ledcSetup(3, 2000, 8); /* 2000 hz PWM, 8-bit resolution and range from 0 to 255 */
  ledcSetup(4, 2000, 8); /* 2000 hz PWM, 8-bit resolution and range from 0 to 255 */
  ledcSetup(5, 2000, 8); /* 2000 hz PWM, 8-bit resolution and range from 0 to 255 */
  ledcSetup(6, 2000, 8); /* 2000 hz PWM, 8-bit resolution and range from 0 to 255 */
  ledcAttachPin(LEFT_M0, 3);//IO13
  ledcAttachPin(LEFT_M1, 4);//IO12
  ledcAttachPin(RIGHT_M0, 5);//IO14
  ledcAttachPin(RIGHT_M1, 6);//IO15
  
  pinMode(33, OUTPUT);
  robot_stop();  
}

// Motor Control Functions

void update_speed()
{  
    ledcWrite(motorPWMChannnel, get_speed(motor_speed));
}

void robot_stop()
{
  ledcWrite(3, 0);
  ledcWrite(4, 0);
  ledcWrite(5, 0);
  ledcWrite(6, 0);
}

void robot_fwd()
{
  ledcWrite(3, 0);
  ledcWrite(4, speed);
  ledcWrite(5, 0);
  ledcWrite(6, speed);
}

void robot_back()
{
  ledcWrite(3, speed);
  ledcWrite(4, 0);
  ledcWrite(5, speed);
  ledcWrite(6, 0);
}

void robot_right()
{
  ledcWrite(3, 0);
  ledcWrite(4, speed);
  ledcWrite(5, speed);
  ledcWrite(6, 0);
}

void robot_left()
{
  ledcWrite(3, speed);
  ledcWrite(4, 0);
  ledcWrite(5, 0);
  ledcWrite(6, speed);
}

extern int gpLed;
extern String WiFiAddr;

void WheelAct(int nLf, int nLb, int nRf, int nRb);

typedef struct {
        size_t size; //number of values used for filtering
        size_t index; //current value index
        size_t count; //value count
        int sum;
        int * values; //array to be filled with values
} ra_filter_t;

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size){
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if(!filter->values){
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t * filter, int value){
    if(!filter->values){
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size) {
        filter->count++;
    }
    return filter->sum / filter->count;
}

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t capture_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.printf("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

    size_t fb_len = 0;
    if(fb->format == PIXFORMAT_JPEG){
        fb_len = fb->len;
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
        jpg_chunking_t jchunk = {req, 0};
        res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
        httpd_resp_send_chunk(req, NULL, 0);
        fb_len = jchunk.len;
    }
    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    Serial.printf("JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
    return res;
}

static esp_err_t stream_handler(httpd_req_t *req){
    if (!live_feed_enabled) {
        return httpd_resp_send(req, "Live feed is off", HTTPD_RESP_USE_STRLEN);
    }

    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];

    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(live_feed_enabled){
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.printf("Camera capture failed");
            res = ESP_FAIL;
        } else {
            if(fb->format != PIXFORMAT_JPEG){
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if(!jpeg_converted){
                    Serial.printf("JPEG compression failed");
                    res = ESP_FAIL;
                }
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        Serial.printf("MJPG: %uB %ums (%.1ffps)\n",
            (uint32_t)(_jpg_buf_len),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    }

    last_frame = 0;
    return res;
}

// Toggle live feed on/off
static esp_err_t toggle_feed_handler(httpd_req_t *req) {
    char buf[5];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0) {
        return ESP_FAIL;
    }

    // Expecting "on" or "off"
    if (strncmp(buf, "on", 2) == 0) {
        live_feed_enabled = true;
    } else if (strncmp(buf, "off", 3) == 0) {
        live_feed_enabled = false;
    } else {
        return ESP_FAIL;
    }

    return httpd_resp_send(req, "Feed toggled", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cmd_handler(httpd_req_t *req){
    char buf[128];
    int ret,cmd;
    ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0) {
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    sscanf(buf, "%d", &cmd);
    Serial.printf("cmd = %d\n", cmd);
    switch(cmd){
        case 0: robot_stop(); break;
        case 1: robot_fwd(); break;
        case 2: robot_back(); break;
        case 3: robot_left(); break;
        case 4: robot_right(); break;
    }
    return httpd_resp_send(req, "Command executed", HTTPD_RESP_USE_STRLEN);
}

void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t cmd_uri = {
        .uri       = "/cmd",
        .method    = HTTP_POST,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t toggle_feed_uri = {
        .uri       = "/toggle_feed",
        .method    = HTTP_POST,
        .handler   = toggle_feed_handler,
        .user_ctx  = NULL
    };

    // Starting the server
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        httpd_register_uri_handler(stream_httpd, &cmd_uri);
        httpd_register_uri_handler(stream_httpd, &toggle_feed_uri);
    }
}
