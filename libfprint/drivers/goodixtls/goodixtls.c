// Goodix Tls driver for libfprint

// Copyright (C) 2021 Alexander Meiler <alex.meiler@protonmail.com>
// Copyright (C) 2021 Matthieu CHARETTE <matthieu.charette@gmail.com>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <gio/gio.h>
#include <glib.h>
#include <netinet/in.h>
#include <openssl/crypto.h>
#include <openssl/err.h>
#include <openssl/ssl.h>
#include <openssl/tls1.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "drivers_api.h"
#include "fp-device.h"
#include "fpi-device.h"
#include "glibconfig.h"
#include "goodix.h"
#include "goodixtls.h"

// The queue can be empty, and then there is no reason string.
static GError* err_from_ssl(const char* what)
{
    unsigned long code = ERR_get_error();
    const char* reason = code ? ERR_reason_error_string(code) : NULL;

    return g_error_new(G_IO_ERROR, G_IO_ERROR_FAILED, "%s: %s", what,
                       reason ? reason : "no OpenSSL error queued");
}

static unsigned int tls_server_psk_server_callback(SSL *ssl,
                                                   const char *identity,
                                                   unsigned char *psk,
                                                   unsigned int max_psk_len) {
  if (sizeof(goodix_511_psk_0) > max_psk_len) {
    fp_dbg("Provided PSK R is too long for OpenSSL");
    return 0;
  }
  fp_dbg("PSK WANTED %d", max_psk_len);
  // I don't know why we must use OPENSSL_hexstr2buf but just copying zeros
  // doesn't work
  const char* buff = "000000000000000000000000000000000000000000000000000000000"
                     "0000000";
  long len = 0;
  unsigned char* key = OPENSSL_hexstr2buf(buff, &len);
  memcpy(psk, key, len);
  OPENSSL_free(key);

  return len;
}

static SSL_CTX* tls_server_create_ctx(void)
{
    const SSL_METHOD* method;

    method = TLS_server_method();

    SSL_CTX* ctx = SSL_CTX_new(method);
    if (!ctx) {
        return NULL;
    }

    return ctx;
}

static void tls_server_config_ctx(SSL_CTX* ctx)
{
    SSL_CTX_set_ecdh_auto(ctx, 1);
    SSL_CTX_set_dh_auto(ctx, 1);
    SSL_CTX_set_cipher_list(ctx, "ALL");
    SSL_CTX_set_min_proto_version(ctx, TLS1_2_VERSION);
    SSL_CTX_set_max_proto_version(ctx, TLS1_2_VERSION);
    SSL_CTX_set_psk_server_callback(ctx, tls_server_psk_server_callback);
}

int goodix_tls_client_send(GoodixTlsServer* self, guint8* data, guint16 length)
{
    return write(self->client_fd, data, length * sizeof(guint8));
}
int goodix_tls_client_recv(GoodixTlsServer* self, guint8* data, guint16 length) {
    struct pollfd pfd = {.fd = self->client_fd, .events = POLLIN};
    int ready = poll(&pfd, 1, GOODIX_TLS_IO_TIMEOUT_MS);

    if (ready <= 0) {
        if (ready == 0)
            errno = ETIMEDOUT;
        return -1;
    }
    return read(self->client_fd, data, length * sizeof(guint8));
}

int goodix_tls_server_receive(GoodixTlsServer* self, guint8* data,
                              guint32 length, GError** error)
{
    ERR_clear_error();
    int retr = SSL_read(self->ssl_layer, data, length * sizeof(guint8));
    if (retr <= 0) {
        // Every USB packet carries whole records and nothing else feeds the
        // socket, so waiting for the rest of one would never end.
        int ssl_error = SSL_get_error(self->ssl_layer, retr);
        if (ssl_error == SSL_ERROR_WANT_READ ||
            ssl_error == SSL_ERROR_WANT_WRITE)
            *error = g_error_new(G_IO_ERROR, G_IO_ERROR_PARTIAL_INPUT,
                                 "Incomplete TLS record");
        else
            *error = err_from_ssl("TLS read failed");
    }
    return retr;
}

static void tls_config_ssl(SSL* ssl)
{
    SSL_set_min_proto_version(ssl, TLS1_2_VERSION);
    SSL_set_max_proto_version(ssl, TLS1_2_VERSION);
    SSL_set_psk_server_callback(ssl, tls_server_psk_server_callback);
    SSL_set_cipher_list(ssl, "ALL");
}

static void* goodix_tls_init_serve(void* me)
{
    GoodixTlsServer* self = me;

    fp_dbg("TLS server waiting to accept...");
    int retr = SSL_accept(self->ssl_layer);
    fp_dbg("TLS server accept done");
    if (retr <= 0) {
        self->connection_callback(self, err_from_ssl("TLS handshake failed"),
                                  self->user_data);
    }
    else {
        self->connection_callback(self, NULL, self->user_data);
    }
    return NULL;
}

static gboolean set_nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL);

    return flags != -1 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) != -1;
}

gboolean goodix_tls_server_set_nonblocking(GoodixTlsServer* self, GError** error)
{
    if (set_nonblocking(self->sock_fd) && set_nonblocking(self->client_fd))
        return TRUE;

    g_set_error(error, G_IO_ERROR, g_io_error_from_errno(errno),
                "failed to make TLS sockets non-blocking: %s", g_strerror(errno));
    return FALSE;
}

gboolean goodix_tls_server_deinit(GoodixTlsServer* self, GError** error)
{
    SSL_shutdown(self->ssl_layer);
    SSL_free(self->ssl_layer);

    close(self->client_fd);
    close(self->sock_fd);

    SSL_CTX_free(self->ssl_ctx);

    return TRUE;
}

gboolean goodix_tls_server_init(GoodixTlsServer* self, GError** error)
{
    g_assert(self->connection_callback);
    SSL_load_error_strings();
    OpenSSL_add_ssl_algorithms();
    SSL_library_init();
    self->ssl_ctx = tls_server_create_ctx();
    tls_server_config_ctx(self->ssl_ctx);

    int socks[2] = {0, 0};
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, socks) != 0) {
        g_set_error(error, G_FILE_ERROR, errno,
                    "failed to create socket pair: %s", strerror(errno));
        return FALSE;
    }
    self->sock_fd = socks[0];
    self->client_fd = socks[1];

    if (self->ssl_ctx == NULL) {
        fp_dbg("Unable to create TLS server context\n");
        *error = fpi_device_error_new_msg(FP_DEVICE_ERROR_GENERAL, "Unable to "
                                                                   "create TLS "
                                                                   "server "
                                                                   "context");
        return FALSE;
    }
    self->ssl_layer = SSL_new(self->ssl_ctx);
    tls_config_ssl(self->ssl_layer);
    SSL_set_fd(self->ssl_layer, self->sock_fd);

    pthread_create(&self->serve_thread, 0, goodix_tls_init_serve, self);

    return TRUE;
}
