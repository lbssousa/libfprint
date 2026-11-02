#!/usr/bin/env bash
# Registra o consumo do fprintd ao longo do tempo, sem privilégios, para ver se
# a memória ou as threads crescem. Continua rodando com a tela bloqueada.
#
#   fprintd-watch.sh [-i SEGUNDOS] [-o ARQUIVO]
#     -i  intervalo (padrão: 30 segundos)
#     -o  arquivo TSV (padrão: um novo em ~/.cache/libfprint-test)
#
# Ctrl+C encerra e mostra o crescimento. Um reinício do fprintd zera a conta:
# o resumo só compara amostras do último PID.
set -uo pipefail

interval=30
log_dir=${XDG_CACHE_HOME:-$HOME/.cache}/libfprint-test
log=$log_dir/fprintd-watch-$(date +%Y%m%d-%H%M%S).tsv
while getopts i:o: opt; do
    case $opt in
    i) interval=$OPTARG ;;
    o) log=$OPTARG ;;
    *) exit 2 ;;
    esac
done
mkdir -p "$(dirname "$log")"

clock_ticks=$(getconf CLK_TCK)

summary() {
    awk -F'\t' 'NR > 1 && $3 > 0 {
            if ($3 != pid) { pid = $3; t0 = $2; rss0 = $4; thr0 = $5; n = 0 }
            t = $2; rss = $4; thr = $5; n++
        }
        END {
            if (n < 2) { print "Amostras insuficientes do mesmo PID."; exit }
            hours = (t - t0) / 3600
            printf "PID %s, %.2f h: RSS de %d para %d kB (%+d kB", pid, hours, rss0, rss, rss - rss0
            if (hours > 0) printf ", %+.0f kB/h", (rss - rss0) / hours
            printf "); threads de %d para %d\n", thr0, thr
        }' "$log"
    echo "Registro: $log"
    exit 0
}
trap summary INT TERM

printf 'hora\tepoch\tpid\trss_kb\tthreads\tcpu_s\tworkers\tbloqueada\n' >"$log"
echo "Registrando a cada ${interval}s em $log. Ctrl+C encerra."

while true; do
    pid=$(systemctl show fprintd -p MainPID --value 2>/dev/null)
    rss=0 threads=0 cpu=0
    if [[ -n $pid && $pid != 0 && -r /proc/$pid/status ]]; then
        rss=$(awk '/^VmRSS/ {print $2}' "/proc/$pid/status" 2>/dev/null)
        threads=$(awk '/^Threads/ {print $2}' "/proc/$pid/status" 2>/dev/null)
        stat=$(<"/proc/$pid/stat")
        read -ra fields <<<"${stat##*) }"
        cpu=$(((fields[11] + fields[12]) / clock_ticks))
    fi
    workers=$(pgrep -c -u 0 -f '^gdm-(session|password)-worker' || true)
    session=$(loginctl show-user "$USER" -p Display --value 2>/dev/null)
    locked=$(loginctl show-session "$session" -p LockedHint --value 2>/dev/null)
    printf '%(%T)T\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' -1 "$EPOCHSECONDS" "${pid:-0}" "${rss:-0}" \
        "${threads:-0}" "$cpu" "$workers" "${locked:-?}" >>"$log"
    sleep "$interval" &
    wait $!
done
