#!/usr/bin/env bash
# Faz o fprintd carregar um build de teste do libfprint, sem gerar imagem nem
# reiniciar a máquina. Só o fprintd muda: um drop-in define LD_LIBRARY_PATH para
# ele, e o resto do sistema segue com a biblioteca da imagem.
#
#   swap-lib.sh build [REF]   compila REF (padrão: fix/goodixtls-hangs) num
#                             contêiner, como o libfprint-builder da imagem
#   swap-lib.sh install       instala o último build e reinicia o fprintd
#   swap-lib.sh uninstall     volta à biblioteca da imagem
#   swap-lib.sh status        mostra qual biblioteca o fprintd está usando
#
# install e uninstall pedem autenticação (run0). O caminho e o nome do drop-in
# são outros que os da play libfprint do bluefin-dakota-setup de propósito: ela
# roda em todo `just system` e apaga o override.conf antigo.
set -euo pipefail

repo=$(git -C "$(dirname "$(readlink -f "$0")")" rev-parse --show-toplevel)
cache=${XDG_CACHE_HOME:-$HOME/.cache}/libfprint-test
image=localhost/libfprint-build:f44
so_name=libfprint-2.so.2.0.0
soname=libfprint-2.so.2
lib_dir=/var/usrlocal/lib/libfprint-test
dropin=/etc/systemd/system/fprintd.service.d/50-libfprint-test.conf
fprintd=/usr/libexec/fprintd

ensure_image() {
    podman image exists "$image" && return
    echo "Criando a imagem de build $image..."
    podman build -q -t "$image" -f - "$cache" <<'EOF'
FROM registry.fedoraproject.org/fedora:44
RUN dnf install -y meson gcc gcc-c++ ninja-build pkgconf-pkg-config openssl openssl-devel glib2-devel gobject-introspection-devel libgudev-devel libgusb-devel systemd-devel systemd-udev nss-devel pixman-devel gtk-doc python3-cairo python3-gobject cairo-devel umockdev git && dnf clean all
EOF
}

cmd_build() {
    local ref=${1:-fix/goodixtls-hangs} rev
    rev=$(git -C "$repo" rev-parse --verify "$ref^{commit}")

    mkdir -p "$cache"
    ensure_image
    rm -rf "$cache/src" "$cache/build"
    mkdir -p "$cache/src" "$cache/build"
    git -C "$repo" archive "$rev" | tar -x -C "$cache/src"

    podman run --rm -v "$cache/src:/src:ro" -v "$cache/build:/build" "$image" bash -c '
        set -e
        meson setup /build /src -Ddrivers=all -Dgtk-examples=false -Ddoc=false >/dev/null
        ninja -C /build libfprint/libfprint-2.so.2.0.0 tests/test-goodixtls-server >/dev/null
        /build/tests/test-goodixtls-server >/dev/null'

    echo "$rev" >"$cache/build/REVISION"
    echo "Compilado: $(git -C "$repo" log -1 --format='%h %s' "$rev")"
    echo "O teste do transporte TLS passou. Instale com: $0 install"
}

# O que dá para conferir sem privilégios, antes de pedir autenticação.
preflight() {
    local so=$cache/build/libfprint/$so_name staging resolved

    if [[ ! -f $so ]]; then
        echo "Nenhum build encontrado. Rode antes: $0 build" >&2
        exit 1
    fi
    staging=$(mktemp -d)
    trap 'rm -rf "$staging"' RETURN
    ln -s "$so" "$staging/$soname"
    resolved=$(LD_LIBRARY_PATH=$staging ldd "$fprintd" 2>&1)
    if grep -qE 'not found' <<<"$resolved"; then
        echo "O fprintd não carregaria este build:" >&2
        grep -E 'not found' <<<"$resolved" >&2
        exit 1
    fi
    if ! grep -qF "$staging/$soname" <<<"$resolved"; then
        echo "O fprintd não usaria este build: não resolveu $soname nele." >&2
        exit 1
    fi
}

cmd_install() {
    preflight
    # shellcheck disable=SC2016  # expandido pelo bash do run0, com os argumentos
    run0 bash -c '
        set -euo pipefail
        so=$1 so_name=$2 soname=$3 lib_dir=$4 dropin=$5 revision=$6

        remove() {
            rm -f "$dropin"
            rm -rf "$lib_dir"
            systemctl daemon-reload
            systemctl restart fprintd || true
        }

        install -d -m 0755 "$lib_dir" "$(dirname "$dropin")"
        install -m 0755 "$so" "$lib_dir/$so_name"
        ln -sf "$so_name" "$lib_dir/$soname"
        install -m 0644 "$revision" "$lib_dir/REVISION"
        printf "[Service]\nEnvironment=LD_LIBRARY_PATH=%s\n" "$lib_dir" >"$dropin"
        systemctl daemon-reload

        pid=0
        if systemctl restart fprintd; then
            pid=$(systemctl show fprintd -p MainPID --value)
        fi
        if [[ $pid == 0 ]] || ! grep -qF "$lib_dir/$so_name" "/proc/$pid/maps"; then
            echo "O fprintd não subiu com o build de teste; voltando à biblioteca da imagem." >&2
            journalctl -u fprintd -n 15 --no-pager >&2 || true
            remove
            exit 1
        fi
    ' _ "$cache/build/libfprint/$so_name" "$so_name" "$soname" "$lib_dir" "$dropin" \
        "$cache/build/REVISION"

    echo "Instalado: $(git -C "$repo" log -1 --format='%h %s' "$(cat "$cache/build/REVISION")")"
    cmd_status
}

cmd_uninstall() {
    # shellcheck disable=SC2016  # expandido pelo bash do run0, com os argumentos
    run0 bash -c '
        set -euo pipefail
        rm -f "$2"
        rm -rf "$1"
        systemctl daemon-reload
        systemctl restart fprintd
    ' _ "$lib_dir" "$dropin"
    cmd_status
}

# /proc/PID/maps do fprintd só abre para root, mas LD_LIBRARY_PATH está na
# unidade e REVISION junto da biblioteca.
cmd_status() {
    local env
    env=$(systemctl show fprintd -p Environment --value)
    echo "fprintd: $(systemctl is-active fprintd || true), PID $(systemctl show fprintd -p MainPID --value)"
    if [[ $env == *"$lib_dir"* ]]; then
        echo "Biblioteca: build de teste em $lib_dir"
        [[ -r $lib_dir/REVISION ]] &&
            echo "Revisão: $(git -C "$repo" log -1 --format='%h %s' "$(cat "$lib_dir/REVISION")" 2>/dev/null || cat "$lib_dir/REVISION")"
    else
        echo "Biblioteca: a da imagem"
    fi
    fprintd-list "$USER" 2>&1 | grep -E 'found|Fingerprints for|^ - ' || true
}

case ${1:-} in
build) shift; cmd_build "$@" ;;
install) cmd_install ;;
uninstall) cmd_uninstall ;;
status) cmd_status ;;
*)
    sed -n '2,/^set -euo/p' "$0" | grep '^#' | cut -c3-
    exit 2
    ;;
esac
