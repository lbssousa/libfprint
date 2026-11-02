#!/usr/bin/env bash
# Inicia e interrompe fprintd-verify em momentos aleatórios, para exercitar o
# cancelamento do driver, e confere a cada volta se o fprintd ainda responde.
#
#   cancel-stress.sh [-d MINUTOS] [-m SEGUNDOS] [-t N]
#     -d  duração (padrão: 30 minutos)
#     -m  tempo máximo antes de cada interrupção (padrão: 8 segundos)
#     -t  a cada N voltas, pede uma digital de verdade (padrão: 0, nunca)
#
# Rode com a tela desbloqueada. Enquanto roda, o sensor fica ocupado, e sudo e
# polkit caem na YubiKey ou na senha. Termina com status diferente de zero se
# houver travamento, fprintd sem responder ou erro do driver no journal.
set -uo pipefail

minutes=30
max_delay=8
touch_every=0
while getopts d:m:t: opt; do
    case $opt in
    d) minutes=$OPTARG ;;
    m) max_delay=$OPTARG ;;
    t) touch_every=$OPTARG ;;
    *) exit 2 ;;
    esac
done

# Quanto esperar o fprintd-verify sair depois de interrompido
exit_limit=${CANCEL_STRESS_EXIT_LIMIT:-40}
ping_timeout=5
driver_errors='already running|Invalid protocol|Invalid ACK|Command timed out|Receive data error|Incomplete|Unexpected TLS|TLS read failed|segfault|dumped core'

log_dir=${XDG_CACHE_HOME:-$HOME/.cache}/libfprint-test
mkdir -p "$log_dir"
log=$log_dir/cancel-stress-$(date +%Y%m%d-%H%M%S).tsv
started_at=$(date '+%Y-%m-%d %H:%M:%S')
deadline=$((SECONDS + minutes * 60))

rounds=0 hangs=0 ping_failures=0 restarts=0 slowest_ping=0
verify_pid=
first_pid=$(systemctl show fprintd -p MainPID --value)
last_pid=$first_pid

rss_kb() {
    awk '/^VmRSS/ {print $2}' "/proc/$1/status" 2>/dev/null || echo 0
}

# Falso se o fprintd não responder a tempo. Guarda a demora em ping_ms.
ping_fprintd() {
    # O separador decimal de EPOCHREALTIME segue o locale.
    local start=${EPOCHREALTIME/,/.} status
    gdbus call --system --dest net.reactivated.Fprint \
        --object-path /net/reactivated/Fprint/Manager \
        --method net.reactivated.Fprint.Manager.GetDevices \
        --timeout "$ping_timeout" >/dev/null 2>&1
    status=$?
    ping_ms=$(awk -v a="$start" -v b="${EPOCHREALTIME/,/.}" 'BEGIN {printf "%d", (b - a) * 1000}')
    return "$status"
}

# Espera o processo sair, até o limite. Falso se ainda estiver preso.
wait_exit() {
    local pid=$1 limit=$((SECONDS + exit_limit))
    while kill -0 "$pid" 2>/dev/null; do
        ((SECONDS < limit)) || return 1
        sleep 0.2
    done
    wait "$pid" 2>/dev/null
    return 0
}

finish() {
    local fingerprint_errors status=0
    trap - INT TERM
    if [[ -n $verify_pid ]] && kill -0 "$verify_pid" 2>/dev/null; then
        kill -INT "$verify_pid" 2>/dev/null
        wait_exit "$verify_pid" || kill -KILL "$verify_pid" 2>/dev/null
    fi
    fingerprint_errors=$(journalctl -u fprintd --since "$started_at" --no-pager -o cat 2>/dev/null |
        grep -cE "$driver_errors")

    echo
    echo "Voltas: $rounds"
    echo "fprintd-verify presos: $hangs"
    echo "fprintd sem responder: $ping_failures"
    echo "Mudanças de PID do fprintd (quedas): $restarts"
    echo "Resposta mais lenta: ${slowest_ping} ms"
    echo "Erros do driver no journal: $fingerprint_errors"
    if ((fingerprint_errors > 0)); then
        journalctl -u fprintd --since "$started_at" --no-pager -o cat 2>/dev/null |
            grep -E "$driver_errors" | sed -E 's/0x[0-9a-f]+/0x…/g' | sort | uniq -c | sort -rn | head -10
    fi
    echo "RSS do fprintd: $(rss_kb "$first_pid") kB (início: ${first_rss} kB)"
    echo "Registro: $log"

    ((hangs + ping_failures + restarts + fingerprint_errors == 0)) || status=1
    ((status == 0)) && echo "RESULTADO: passou" || echo "RESULTADO: falhou"
    exit "$status"
}

first_rss=$(rss_kb "$first_pid")
trap finish INT TERM
printf 'hora\tvolta\tsinal\tespera_s\tping_ms\tpid\trss_kb\tresultado\n' >"$log"
echo "Rodando por $minutes min. Ctrl+C encerra e mostra o resumo."

while ((SECONDS < deadline)); do
    ((++rounds))
    touch_round=0
    if ((touch_every > 0 && rounds % touch_every == 0)); then
        touch_round=1
        echo "Volta $rounds: encoste o dedo no sensor (20 s)."
    fi

    # Um script inicia processos em segundo plano com SIGINT ignorado, e o
    # bash não sabe desfazer isso.
    python3 -Ic 'import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])' fprintd-verify >/dev/null 2>&1 &
    verify_pid=$!

    # Intercala os dois caminhos: VerifyStop pelo cliente, e cliente que some.
    ((rounds % 2)) && signal=INT || signal=KILL
    if ((touch_round)); then
        delay=20
        signal=INT
    else
        delay=$(awk -v seed="$RANDOM$rounds" -v max="$max_delay" 'BEGIN {srand(seed); printf "%.2f", 0.1 + rand() * (max - 0.1)}')
    fi
    sleep "$delay"
    kill "-$signal" "$verify_pid" 2>/dev/null

    result=ok
    if ! wait_exit "$verify_pid"; then
        result=preso
        ((++hangs))
        echo "Volta $rounds: fprintd-verify não saiu em ${exit_limit}s depois de SIG$signal."
        kill -KILL "$verify_pid" 2>/dev/null
        wait "$verify_pid" 2>/dev/null
    fi

    if ! ping_fprintd; then
        result=sem-resposta
        ((++ping_failures))
        echo "Volta $rounds: o fprintd não respondeu em ${ping_timeout}s."
    fi
    ((ping_ms > slowest_ping)) && slowest_ping=$ping_ms

    pid=$(systemctl show fprintd -p MainPID --value)
    if [[ $pid != "$last_pid" ]]; then
        ((++restarts))
        result=novo-pid
        echo "Volta $rounds: o PID do fprintd mudou de $last_pid para $pid."
        last_pid=$pid
    fi

    printf '%(%T)T\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' -1 "$rounds" "$signal" "$delay" \
        "$ping_ms" "$pid" "$(rss_kb "$pid")" "$result" >>"$log"
done

finish
