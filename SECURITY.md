# Security policy

This repository is a fork of [libfprint](https://gitlab.freedesktop.org/libfprint/libfprint)
that adds the `goodixtls53xd` driver (Goodix 27c6:538d) and the SIGFM matcher.

## Supported versions

Only the latest release tag (`v<upstream>-goodix538d.<n>`) on the
`goodix-538d-sigfm-gtls` branch receives fixes.

## Reporting a vulnerability

- **Fork code** (`libfprint/drivers/goodixtls/`, `libfprint/sigfm/`, `rpm/`):
  report it privately through GitHub's
  [private vulnerability reporting](https://github.com/lbssousa/libfprint/security/advisories/new).
  Please do not open a public issue.
- **Upstream libfprint code**: report it to the upstream project, following
  its own process on gitlab.freedesktop.org.

Please include the affected version or commit, steps to reproduce, and the
impact you expect. You should get a first reply within 7 days.
