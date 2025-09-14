#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <esp_http_server.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// HTTP Server Configuration
#define HTTP_SERVER_PORT                80
#define MAX_URI_HANDLERS                16

// HTTP server configuration - using ESP-IDF's built-in types
// Use httpd_config_t directly from esp_http_server.h

// HTTP request/response context - using ESP-IDF's built-in types
// Use httpd_req_t directly from esp_http_server.h

// Function prototypes

/**
 * @brief Initialize HTTP server
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_init(void);

/**
 * @brief Deinitialize HTTP server
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_deinit(void);

/**
 * @brief Start HTTP server
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_start(void);

/**
 * @brief Stop HTTP server
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_stop(void);

/**
 * @brief Configure HTTP server
 * @param config Pointer to server configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_configure(httpd_config_t *config);

/**
 * @brief Register URI handler
 * @param uri URI pattern
 * @param method HTTP method
 * @param handler Handler function
 * @param user_ctx User context
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_register_uri_handler(const char *uri, httpd_method_t method, 
                                          esp_err_t (*handler)(httpd_req_t *req), void *user_ctx);

/**
 * @brief Unregister URI handler
 * @param uri URI pattern
 * @param method HTTP method
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_unregister_uri_handler(const char *uri, httpd_method_t method);

/**
 * @brief Send HTTP response
 * @param req HTTP request
 * @param content_type Content type
 * @param content Response content
 * @param content_len Content length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_send_response(httpd_req_t *req, const char *content_type, 
                                   const char *content, size_t content_len);

/**
 * @brief Send JSON response
 * @param req HTTP request
 * @param json_data JSON data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_send_json_response(httpd_req_t *req, const char *json_data);

/**
 * @brief Send error response
 * @param req HTTP request
 * @param status_code HTTP status code
 * @param error_msg Error message
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_send_error_response(httpd_req_t *req, int status_code, const char *error_msg);

/**
 * @brief Parse query parameters
 * @param query_string Query string
 * @param params Parameter array
 * @param max_params Maximum parameters
 * @param param_count Actual parameter count
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_parse_query_params(const char *query_string, char **params, 
                                        uint8_t max_params, uint8_t *param_count);

/**
 * @brief Get query parameter value
 * @param query_string Query string
 * @param param_name Parameter name
 * @param param_value Buffer for parameter value
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_get_query_param(const char *query_string, const char *param_name, 
                                     char *param_value, size_t max_len);

/**
 * @brief Parse POST data
 * @param req HTTP request
 * @param post_data Buffer for POST data
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_parse_post_data(httpd_req_t *req, char *post_data, size_t max_len);

/**
 * @brief Parse JSON POST data
 * @param req HTTP request
 * @param json_data Buffer for JSON data
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_parse_json_post_data(httpd_req_t *req, char *json_data, size_t max_len);

/**
 * @brief Set response header
 * @param resp HTTP response
 * @param header_name Header name
 * @param header_value Header value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_set_response_header(httpd_req_t *req, const char *header_name,
                                         const char *header_value);

/**
 * @brief Set CORS headers
 * @param resp HTTP response
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_set_cors_headers(httpd_req_t *req);

/**
 * @brief Enable CORS for all routes
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_enable_cors(bool enable);

/**
 * @brief Get server statistics
 * @param stats Pointer to server statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_get_stats(void *stats);

/**
 * @brief Reset server statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_reset_stats(void);

// Default URI handlers

/**
 * @brief Root URI handler
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_root_handler(httpd_req_t *req);

/**
 * @brief API status handler
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_api_status_handler(httpd_req_t *req);

/**
 * @brief API control handler
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_api_control_handler(httpd_req_t *req);

/**
 * @brief API configuration handler
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_api_config_handler(httpd_req_t *req);

/**
 * @brief API sensor data handler
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_api_sensor_handler(httpd_req_t *req);

/**
 * @brief WebSocket handler
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_websocket_handler(httpd_req_t *req);

/**
 * @brief File upload handler
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_file_upload_handler(httpd_req_t *req);

/**
 * @brief File download handler
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_file_download_handler(httpd_req_t *req);

// Web interface functions

/**
 * @brief Serve web interface files
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_serve_web_interface(httpd_req_t *req);

/**
 * @brief Generate web interface HTML
 * @param html_buffer Buffer for HTML content
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_generate_web_interface(char *html_buffer, size_t max_len);

/**
 * @brief Update web interface data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t http_server_update_web_interface(void);

// Default configuration
#define DEFAULT_HTTP_SERVER_PORT             80
#define DEFAULT_HTTP_SERVER_MAX_URI_HANDLERS 8
#define DEFAULT_HTTP_SERVER_MAX_RESP_HEADERS 4
#define DEFAULT_HTTP_SERVER_MAX_OPEN_SOCKETS 7
#define DEFAULT_HTTP_SERVER_MAX_URI_LEN     512
#define DEFAULT_HTTP_SERVER_MAX_POST_LEN    1024
#define DEFAULT_HTTP_SERVER_MAX_HEADER_LEN  512
#define DEFAULT_HTTP_SERVER_KEEP_ALIVE_IDLE 5
#define DEFAULT_HTTP_SERVER_KEEP_ALIVE_INTERVAL 5
#define DEFAULT_HTTP_SERVER_KEEP_ALIVE_COUNT 3

// URI patterns
#define HTTP_URI_ROOT              "/"
#define HTTP_URI_API_STATUS        "/api/status"
#define HTTP_URI_API_CONTROL       "/api/control"
#define HTTP_URI_API_CONFIG        "/api/config"
#define HTTP_URI_API_SENSOR        "/api/sensor"
#define HTTP_URI_WEBSOCKET         "/ws"
#define HTTP_URI_FILE_UPLOAD       "/upload"
#define HTTP_URI_FILE_DOWNLOAD     "/download"

// Content types
#define HTTP_CONTENT_TYPE_HTML     "text/html"
#define HTTP_CONTENT_TYPE_JSON     "application/json"
#define HTTP_CONTENT_TYPE_TEXT     "text/plain"
#define HTTP_CONTENT_TYPE_CSS      "text/css"
#define HTTP_CONTENT_TYPE_JS       "application/javascript"
#define HTTP_CONTENT_TYPE_BINARY   "application/octet-stream"

#ifdef __cplusplus
}
#endif

#endif // HTTP_SERVER_H
