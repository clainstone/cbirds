# Trovare il collo di bottiglia con 4.096 boids

Tutti i comandi vanno eseguiti dalla radice del repository.

## 1. Compilare il binario di profiling

```bash
make -B TARGET=cbirds-prof \
  CFLAGS='-std=c99 -Wall -Wextra -O3 -g -fno-omit-frame-pointer'
```

Il binario conserva le ottimizzazioni reali (`-O3`) e aggiunge simboli e frame
pointer per rendere leggibili i profili.

## 2. Misurare il tempo reale di partenza

```bash
./cbirds-prof --bench 1000 --seed 1 --render kitty -n 4096
```

Conservare il valore `frame time`: servirà per verificare il guadagno dopo le
modifiche. Il seed fisso rende confrontabili le esecuzioni.

## 3. Raccogliere il profilo Callgrind

```bash
valgrind --tool=callgrind \
  --callgrind-out-file=callgrind.4096.kitty.out \
  ./cbirds-prof --bench 100 --seed 1 --render kitty -n 4096
```

Callgrind rallenta molto il programma: è normale. Se il comando dura troppo,
usare 30 frame; se termina rapidamente, portarlo a 300. Non considerare il
`frame time` stampato durante questa esecuzione come un tempo reale.

## 4. Vedere dove vengono spese le istruzioni

Prima mostrare il costo proprio delle funzioni, cioè il lavoro fatto direttamente
da ognuna:

```bash
callgrind_annotate \
  --inclusive=no \
  --threshold=99 \
  --auto=yes \
  callgrind.4096.kitty.out | less
```

Poi mostrare il costo inclusivo e la catena chiamante/chiamata:

```bash
callgrind_annotate \
  --inclusive=yes \
  --tree=both \
  --threshold=99 \
  callgrind.4096.kitty.out | less
```

Nel report, `Ir` è il numero di istruzioni eseguite. Le percentuali più alte nel
primo report identificano le funzioni sulle quali intervenire; il secondo report
spiega da quale percorso vengono raggiunte. In particolare cercare:

- `spatial_grid_build` e `update_birds`: costo della simulazione;
- `queue_render_frame` e `kitty_graphics_place`: costo del rendering Kitty;
- `hunt`: costo dei falchi, se il test viene ripetuto con `--hawks`.

Con KCachegrind/QCachegrind lo stesso profilo si può esplorare graficamente:

```bash
kcachegrind callgrind.4096.kitty.out
```

## 5. Controllare cache e branch prediction

Se il primo report indica una funzione pesante ma non chiarisce il motivo,
raccogliere un profilo più dettagliato e più lento:

```bash
valgrind --tool=callgrind \
  --cache-sim=yes \
  --branch-sim=yes \
  --callgrind-out-file=callgrind.4096.cache.out \
  ./cbirds-prof --bench 30 --seed 1 --render kitty -n 4096

callgrind_annotate \
  --show=Ir,Dr,Dw,D1mr,D1mw,DLmr,DLmw,Bcm \
  --inclusive=no \
  --threshold=99 \
  --auto=yes \
  callgrind.4096.cache.out | less
```

`D1mr` e `D1mw` sono i miss della cache dati L1, `DLmr` e `DLmw` quelli
dell'ultimo livello, mentre `Bcm` indica branch condizionali predetti male.

## 6. Verificare ogni ottimizzazione

Dopo una modifica, ripetere il benchmark reale con gli stessi parametri:

```bash
./cbirds-prof --bench 1000 --seed 1 --render kitty -n 4096
```

Confrontare `frame time` con il valore iniziale. Rigenerare Callgrind solo dopo
aver osservato un miglioramento reale: il conteggio di istruzioni aiuta a trovare
il collo di bottiglia, ma non sostituisce la misura fuori da Valgrind.
