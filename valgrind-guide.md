# Profilare cbirds con Valgrind

## 1. Compilare un binario profilabile

```bash
make -B TARGET=cbirds-prof \
  CFLAGS='-std=c99 -Wall -Wextra -O3 -g -fno-omit-frame-pointer'
```

Questo crea `cbirds-prof` senza sostituire il normale `cbirds`.

## 2. Raccogliere il profilo

```bash
valgrind --tool=callgrind \
  --callgrind-out-file=callgrind.kitty.out \
  ./cbirds-prof --bench 300 --seed 1 --render kitty
```

Valgrind rallenta molto l'esecuzione: è normale. Se serve, ridurre il numero di
frame. Non usare il `frame time` misurato sotto Valgrind come prestazione reale.

## 3. Leggere gli hotspot

Costo inclusivo, comprese le funzioni chiamate:

```bash
callgrind_annotate --inclusive=yes --threshold=95 --auto=yes \
  callgrind.kitty.out | less
```

Costo interno di ogni funzione:

```bash
callgrind_annotate --inclusive=no --threshold=95 \
  callgrind.kitty.out | less
```

Nel report, `Ir` indica approssimativamente il numero di istruzioni CPU
eseguite. Conviene partire dalle funzioni con le percentuali maggiori.

Se disponibile, il report può essere aperto graficamente:

```bash
kcachegrind callgrind.kitty.out
```

## 4. Verificare il miglioramento reale

Dopo una modifica, misurare fuori da Valgrind mantenendo identici carico e seed:

```bash
./cbirds-prof --bench 3000 --seed 1 --render kitty
```

Per confrontare altri renderer, cambiare esplicitamente `--render`, usando un
file di output diverso per ogni profilo.
