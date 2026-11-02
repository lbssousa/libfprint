/*
 * Unit tests for the goodixtls TLS server: what the main loop gets when the
 * data coming from the sensor is not what it should be.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <errno.h>
#include <gio/gio.h>
#include <glib.h>
#include <openssl/err.h>
#include <openssl/ssl.h>
#include <pthread.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "goodixtls.h"

/* A test that blocks on a socket would otherwise hang instead of failing. */
#define HANG_LIMIT_SECONDS 30

typedef struct
{
  GoodixTlsServer server;
  SSL_CTX        *client_ctx;
  SSL            *client;
} Fixture;

static void
on_server_ready (GoodixTlsServer *server, GError *error, gpointer user_data)
{
  g_clear_error (&error);
}

static unsigned int
client_psk_cb (SSL *ssl, const char *hint, char *identity,
               unsigned int max_identity_len, unsigned char *psk,
               unsigned int max_psk_len)
{
  g_assert_cmpuint (max_psk_len, >=, 32);
  memset (psk, 0, 32);
  g_strlcpy (identity, "test", max_identity_len);
  return 32;
}

static void
fixture_init_server (Fixture *f)
{
  g_autoptr(GError) error = NULL;

  memset (f, 0, sizeof (*f));
  f->server.connection_callback = on_server_ready;
  g_assert_true (goodix_tls_server_init (&f->server, &error));
  g_assert_no_error (error);
}

static void
fixture_setup_connected (Fixture *f, gconstpointer data)
{
  g_autoptr(GError) error = NULL;

  fixture_init_server (f);

  f->client_ctx = SSL_CTX_new (TLS_client_method ());
  g_assert_nonnull (f->client_ctx);
  SSL_CTX_set_min_proto_version (f->client_ctx, TLS1_2_VERSION);
  SSL_CTX_set_max_proto_version (f->client_ctx, TLS1_2_VERSION);
  g_assert_cmpint (SSL_CTX_set_cipher_list (f->client_ctx, "PSK"), ==, 1);
  SSL_CTX_set_psk_client_callback (f->client_ctx, client_psk_cb);

  f->client = SSL_new (f->client_ctx);
  SSL_set_fd (f->client, f->server.client_fd);
  g_assert_cmpint (SSL_connect (f->client), ==, 1);

  g_assert_true (goodix_tls_server_set_nonblocking (&f->server, &error));
  g_assert_no_error (error);
}

static void
fixture_teardown (Fixture *f, gconstpointer data)
{
  g_autoptr(GError) error = NULL;

  g_assert_true (goodix_tls_server_deinit (&f->server, &error));
  g_assert_no_error (error);
  g_clear_pointer (&f->client, SSL_free);
  g_clear_pointer (&f->client_ctx, SSL_CTX_free);
}

static void
test_whole_record (Fixture *f, gconstpointer data)
{
  g_autoptr(GError) error = NULL;
  const char message[] = "image";
  guint8 buffer[64];

  g_assert_cmpint (SSL_write (f->client, message, sizeof (message)), ==, sizeof (message));

  g_assert_cmpint (goodix_tls_server_receive (&f->server, buffer, sizeof (buffer), &error),
                   ==, sizeof (message));
  g_assert_no_error (error);
  g_assert_cmpstr ((const char *) buffer, ==, message);
}

/* This used to block forever, and fprintd with it. */
static void
test_incomplete_record (Fixture *f, gconstpointer data)
{
  g_autoptr(GError) error = NULL;
  const guint8 start_of_record[] = { 0x17, 0x03, 0x03, 0x10, 0x00, 0xaa };
  guint8 buffer[64];

  g_assert_cmpint (goodix_tls_client_send (&f->server, (guint8 *) start_of_record,
                                           sizeof (start_of_record)),
                   ==, sizeof (start_of_record));

  g_assert_cmpint (goodix_tls_server_receive (&f->server, buffer, sizeof (buffer), &error),
                   <=, 0);
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_PARTIAL_INPUT);
}

static void
test_nothing_to_read (Fixture *f, gconstpointer data)
{
  g_autoptr(GError) error = NULL;
  guint8 buffer[64];

  g_assert_cmpint (goodix_tls_server_receive (&f->server, buffer, sizeof (buffer), &error),
                   <=, 0);
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_PARTIAL_INPUT);
}

static void
test_end_of_file (Fixture *f, gconstpointer data)
{
  g_autoptr(GError) error = NULL;
  guint8 buffer[64];

  g_assert_cmpint (shutdown (f->server.client_fd, SHUT_WR), ==, 0);

  g_assert_cmpint (goodix_tls_server_receive (&f->server, buffer, sizeof (buffer), &error),
                   <=, 0);
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_FAILED);
  g_assert_nonnull (error->message);
}

/* A record the TLS session can't authenticate, as after one was dropped. */
static void
test_corrupt_record (Fixture *f, gconstpointer data)
{
  g_autoptr(GError) error = NULL;
  guint8 record[5 + 64] = { 0x17, 0x03, 0x03, 0x00, 64 };
  guint8 buffer[64];

  g_assert_cmpint (goodix_tls_client_send (&f->server, record, sizeof (record)),
                   ==, sizeof (record));

  g_assert_cmpint (goodix_tls_server_receive (&f->server, buffer, sizeof (buffer), &error),
                   <=, 0);
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_FAILED);
}

static void
test_client_recv_timeout (Fixture *f, gconstpointer data)
{
  guint8 buffer[64];

  errno = 0;
  g_assert_cmpint (goodix_tls_client_recv (&f->server, buffer, sizeof (buffer)), ==, -1);
  g_assert_cmpint (errno, ==, ETIMEDOUT);
}

/* The server thread is waiting for a client hello that will never come. */
static void
test_deinit_during_handshake (void)
{
  g_autoptr(GError) error = NULL;
  Fixture f;

  fixture_init_server (&f);

  g_assert_true (goodix_tls_server_deinit (&f.server, &error));
  g_assert_no_error (error);
}

static void
test_deinit_mid_handshake (void)
{
  g_autoptr(GError) error = NULL;
  const guint8 start_of_hello[] = { 0x16, 0x03, 0x01, 0x10, 0x00, 0x01 };
  Fixture f;

  fixture_init_server (&f);
  g_assert_cmpint (goodix_tls_client_send (&f.server, (guint8 *) start_of_hello,
                                           sizeof (start_of_hello)),
                   ==, sizeof (start_of_hello));

  g_assert_true (goodix_tls_server_deinit (&f.server, &error));
  g_assert_no_error (error);
}

#define CONNECTED_TEST(path, func) \
  g_test_add (path, Fixture, NULL, fixture_setup_connected, func, fixture_teardown)

int
main (int argc, char *argv[])
{
  g_test_init (&argc, &argv, NULL);

  alarm (HANG_LIMIT_SECONDS);

  CONNECTED_TEST ("/goodixtls/receive/whole-record", test_whole_record);
  CONNECTED_TEST ("/goodixtls/receive/incomplete-record", test_incomplete_record);
  CONNECTED_TEST ("/goodixtls/receive/nothing-to-read", test_nothing_to_read);
  CONNECTED_TEST ("/goodixtls/receive/end-of-file", test_end_of_file);
  CONNECTED_TEST ("/goodixtls/receive/corrupt-record", test_corrupt_record);
  CONNECTED_TEST ("/goodixtls/client-recv/timeout", test_client_recv_timeout);
  g_test_add_func ("/goodixtls/deinit/during-handshake", test_deinit_during_handshake);
  g_test_add_func ("/goodixtls/deinit/mid-handshake", test_deinit_mid_handshake);

  return g_test_run ();
}
