# Piano di implementazione dello spatial hashing per Cbirds

## 1. Obiettivo

Sostituire la ricerca dei vicini a forza bruta, attualmente `O(n²)`, con una
griglia spaziale uniforme ricostruita a ogni frame.

Il comportamento visivo deve restare quello del modello Boids attuale:

- separazione, allineamento, coesione ed evitamento dei bordi non cambiano;
- il campo visivo continua a essere circolare;
- tutti i boid vengono aggiornati a partire dallo stesso snapshot;
- posizione, direzione e sprite restano sincronizzati;
- `P` e `p` continuano a modificare il raggio di percezione durante
  l'esecuzione.

La griglia serve soltanto a ridurre il numero di candidati. Non crea gruppi
persistenti e non cambia le regole con cui si forma lo stormo.

## 2. Decisioni di progetto

### 2.1 Dimensione fissa delle celle

Ogni cella misura:

```c
SPATIAL_CELL_SIZE = 12 /* pixel */
```

La misura è espressa in pixel e non dipende:

- dalla dimensione in caratteri del terminale;
- dalla dimensione dello sprite scelta con `-s`;
- dal frame rate;
- dal numero di boid.

Dodici pixel sono stati scelti perché rappresentano un buon compromesso fra
granularità e costo della scansione. Il raggio attuale di 35 pixel viene
approssimato quasi esattamente da tre celle, cioè 36 pixel.

### 2.2 Raggio espresso in celle

La configurazione passa da un raggio arbitrario in pixel a un numero intero
di celle:

```c
DEFAULT_VISION_CELLS = 3
MIN_VISION_CELLS = 1
MAX_VISION_CELLS = 12
```

Il raggio effettivo è derivato:

```c
vision_radius_pixels = vision_cells * SPATIAL_CELL_SIZE;
vision_radius_squared = vision_radius_pixels * vision_radius_pixels;
```

Valori principali:

| Celle di visione | Raggio effettivo | Finestra massima visitata |
|---:|---:|---:|
| 1 | 12 px | 3×3, 9 celle |
| 2 | 24 px | 5×5, 25 celle |
| 3, default | 36 px | 7×7, 49 celle |
| 4 | 48 px | 9×9, 81 celle |
| 8 | 96 px | 17×17, 289 celle |
| 12 | 144 px | 25×25, 625 celle |

La finestra indica il massimo numero di celle candidate prima di tagliarla ai
bordi della griglia. I boid vengono comunque filtrati con la distanza
euclidea esatta; il campo visivo non diventa quadrato.

### 2.3 Controlli interattivi

- `P`: aumenta `vision_cells` di uno, fino a 12;
- `p`: diminuisce `vision_cells` di uno, fino a 1.

Ogni pressione cambia quindi il raggio di 12 pixel e modifica immediatamente
il numero di celle adiacenti visitate.

Il raggio e il suo quadrato devono essere aggiornati da una singola funzione,
per evitare che i tre valori derivati diventino incoerenti.

## 3. Struttura dei dati

I boid rimangono organizzati come oggi in due array contigui di `bird_t`:

```text
birds:    stato corrente e modificabile
snapshot: stato immutabile usato per calcolare il frame successivo
```

La griglia non possiede né duplica i `bird_t`: contiene soltanto indici che
puntano dentro `snapshot`.

La struttura prevista è:

```c
typedef struct {
    int cell_size;
    int columns;
    int rows;
    int cell_count;
    int bird_capacity;

    int *counts;   /* occupazione, poi cursore di scrittura */
    int *offsets;  /* cell_count + 1 elementi */
    int *indices;  /* bird_capacity elementi */
} spatial_grid_t;
```

Significato degli array:

- `counts[cell]`: numero di boid assegnati alla cella;
- `offsets[cell]`: posizione iniziale della cella dentro `indices`;
- `offsets[cell + 1]`: posizione successiva all'ultimo indice della cella;
- `indices`: indici dei boid raggruppati in intervalli contigui per cella.

Esempio:

```text
offsets: [0, 2, 2, 5]
indices: [7, 9, 1, 4, 8]

cella 0 -> indici [7, 9]
cella 1 -> nessun boid
cella 2 -> indici [1, 4, 8]
```

Non verranno usate liste concatenate o un'allocazione per boid. Gli intervalli
contigui riducono pointer chasing, frammentazione e allocazioni nel main loop.

## 4. Dimensionamento della griglia

Il numero di celle dipende dalle dimensioni in pixel dello schermo:

```c
columns = (screen.width  + SPATIAL_CELL_SIZE - 1) / SPATIAL_CELL_SIZE;
rows    = (screen.height + SPATIAL_CELL_SIZE - 1) / SPATIAL_CELL_SIZE;
```

Per uno schermo da 640×384 pixel:

```text
columns = 54
rows = 32
cell_count = 1728
```

La griglia viene riallocata soltanto quando:

- cambiano il numero di righe o colonne in seguito a un resize;
- la capacità degli indici non è sufficiente per `config.birds`.

Le moltiplicazioni e le dimensioni delle allocazioni devono essere controllate
prima di chiamare `malloc` o `calloc`. In caso di fallimento, il programma deve
stampare un errore, ripristinare il terminale tramite il percorso già esistente
e terminare senza usare una griglia parzialmente inizializzata.

Una riallocazione deve essere transazionale: i vecchi buffer vengono liberati
solo dopo che tutti i nuovi buffer sono stati allocati con successo.

## 5. Mappatura di una posizione

La cella di un boid si calcola con:

```c
cell_x = (int)floor(bird->x / SPATIAL_CELL_SIZE);
cell_y = (int)floor(bird->y / SPATIAL_CELL_SIZE);
```

Le coordinate vengono poi limitate alla griglia:

```c
cell_x = clamp(cell_x, 0, grid.columns - 1);
cell_y = clamp(cell_y, 0, grid.rows - 1);
```

Questo gestisce anche i boid temporaneamente fuori dallo schermo. Più boid
esterni possono finire nella stessa cella di bordo, ma il successivo controllo
della distanza elimina i falsi candidati. Il clamping non introduce falsi
negativi: due boid realmente vicini non vengono separati in celle più lontane.

La conversione finale è:

```c
cell_index = cell_y * grid.columns + cell_x;
```

## 6. Costruzione della griglia a ogni frame

La griglia viene ricostruita dallo `snapshot` in tre passaggi lineari.

### Passaggio 1: conteggio

```c
memset(grid.counts, 0, grid.cell_count * sizeof(*grid.counts));

for each bird in snapshot:
    cell = cell_for_position(bird.x, bird.y)
    grid.counts[cell]++
```

### Passaggio 2: prefix sum

```c
grid.offsets[0] = 0;

for each cell:
    grid.offsets[cell + 1] = grid.offsets[cell] + grid.counts[cell]
```

Al termine, `offsets[cell]..offsets[cell + 1]` rappresenta l'intervallo
riservato a quella cella.

### Passaggio 3: inserimento degli indici

`counts` può essere azzerato e riutilizzato come cursore temporaneo:

```c
memset(grid.counts, 0, grid.cell_count * sizeof(*grid.counts));

for each bird index i:
    cell = cell_for_position(snapshot[i].x, snapshot[i].y)
    slot = grid.offsets[cell] + grid.counts[cell]++
    grid.indices[slot] = i
```

Non avvengono allocazioni durante questi tre passaggi.

## 7. Ricerca dei vicini

Per il boid target:

1. si calcola la sua cella centrale;
2. si costruisce l'intervallo di celle `±vision_cells`;
3. si taglia l'intervallo ai bordi della griglia;
4. si visitano gli indici contenuti nelle celle;
5. si scarta il target stesso;
6. si applica il controllo preciso sulla distanza al quadrato;
7. si accumulano le regole Boids esistenti.

Pseudocodice:

```c
radius = config.vision_cells;
center = cell_for_position(target.x, target.y);

min_x = max(0, center.x - radius);
max_x = min(grid.columns - 1, center.x + radius);
min_y = max(0, center.y - radius);
max_y = min(grid.rows - 1, center.y + radius);

for cell_y from min_y to max_y:
    for cell_x from min_x to max_x:
        cell = cell_y * grid.columns + cell_x

        for slot from offsets[cell] to offsets[cell + 1]:
            other_index = indices[slot]
            if other_index == target_index:
                continue

            dx = target.x - snapshot[other_index].x
            dy = target.y - snapshot[other_index].y

            if dx*dx + dy*dy >= vision_radius_squared:
                continue

            accumulate separation, alignment and cohesion
```

La formula per separazione, allineamento e coesione rimane invariata. Anche il
confronto sul bordo del raggio mantiene la semantica attuale: una distanza
esattamente uguale al raggio non è considerata visibile.

## 8. Integrazione nel ciclo del frame

Il flusso previsto dentro il main loop è:

```text
1. Gestisci l'input
2. Leggi le nuove dimensioni del terminale
3. Assicura che la griglia abbia dimensioni e capacità corrette
4. Copia birds in snapshot
5. Ricostruisci la griglia usando snapshot
6. Accoda il clear globale e il rendering dello stato corrente
7. Calcola i nuovi boid consultando la griglia
8. Aggiorna lo sprite dalla nuova direzione
9. Completa il flush non bloccante dei comandi Kitty usando `poll()`
10. Rispetta il frame budget
```

L'ordine fra snapshot, costruzione della griglia e aggiornamento è vincolante.
La griglia non deve mai essere costruita da `birds` mentre lo stesso array viene
modificato, altrimenti l'esito dipenderebbe dall'ordine dei boid nell'array.
Se l'output restituisce backpressure, il suffisso non inviato resta nel buffer.
`poll()` attende contemporaneamente capacità sull'output e nuovi tasti, senza
sleep fissi: il loop resta reattivo ma non accoda né simula un altro frame. In
questo modo esiste al massimo un frame pendente e la memoria non cresce con un
terminale più lento del frame rate richiesto.

Le firme concettuali diventano:

```c
static void spatial_grid_build(spatial_grid_t *grid, const bird_t *snapshot);

static double flock_direction(const bird_t *snapshot,
                              const spatial_grid_t *grid,
                              int target_index);

static void update_birds(bird_t *birds,
                         const bird_t *snapshot,
                         const spatial_grid_t *grid);
```

## 9. Modifiche alla configurazione

In `config_t`:

```c
int vision_cells;
int vision_radius;
int vision_radius_squared;
```

Sostituiscono:

```c
int perception_radius;
int perception_radius_squared;
```

Una funzione centralizza i valori derivati:

```c
static void update_vision_radius(void) {
    config.vision_radius = config.vision_cells * SPATIAL_CELL_SIZE;
    config.vision_radius_squared = config.vision_radius * config.vision_radius;
}
```

Con il massimo concordato il quadrato vale al massimo `144² = 20736`, quindi
non presenta rischi di overflow per un `int` ordinario.

## 10. Gestione della memoria

La memoria aggiuntiva è `O(celle + boid)`:

```text
counts:  cell_count interi
offsets: cell_count + 1 interi
indices: config.birds interi
```

Nell'esempio 640×384 con 4096 boid:

- `counts`: circa 6,8 KiB;
- `offsets`: circa 6,8 KiB;
- `indices`: 16 KiB;
- totale griglia: circa 30 KiB.

La griglia deve avere funzioni dedicate per:

```c
spatial_grid_init
spatial_grid_resize
spatial_grid_build
spatial_grid_destroy
```

Per la prima implementazione queste funzioni possono rimanere private in
`boids.c`, perché dipendono direttamente da `bird_t` e `screen_t`. Se in futuro
la simulazione viene separata dal frontend terminale, la griglia potrà essere
spostata in `spatial_grid.c/.h` senza cambiare l'algoritmo.

## 11. Complessità attesa

### Implementazione attuale

```text
Costruzione struttura: nessuna
Ricerca: n × n
Totale: O(n²)
```

Con 4096 boid si eseguono circa 16,7 milioni di confronti fra coppie per frame.

### Implementazione con griglia

```text
Costruzione griglia: O(n + celle)
Ricerca: O(n × candidati locali)
Totale medio: O(n + coppie locali)
```

Il caso peggiore resta `O(n²)`: se tutti i boid occupano poche celle, tutti
rimangono candidati. La griglia accelera il caso normale, non può eliminare il
costo di uno stormo completamente sovrapposto.

Il costo cresce deliberatamente aumentando `vision_cells`, perché ogni boid
deve osservare una porzione più ampia dello spazio.

## 12. Correttezza e casi limite

Devono essere coperti esplicitamente:

- terminale più piccolo di una cella;
- dimensioni terminale non multiple di 12;
- resize durante la simulazione;
- boid con coordinate negative;
- boid oltre il bordo destro o inferiore;
- celle vuote;
- tutti i boid nella stessa cella;
- un solo boid;
- raggio minimo e massimo;
- boid esattamente sul confine fra due celle;
- boid a distanza esattamente uguale al raggio;
- fallimento delle allocazioni della griglia;
- assenza di duplicati nella scansione dei candidati.

La griglia deve essere soltanto un indice. Per ogni configurazione, il risultato
della ricerca deve coincidere con quello della scansione completa, a parità di
snapshot e raggio effettivo.

## 13. Strategia di test

### 13.1 Test deterministici della griglia

Preparare piccoli scenari con coordinate note e verificare:

- assegnazione corretta alle celle;
- prefix sum e intervalli corretti;
- ogni indice presente una sola volta;
- clamping delle coordinate fuori schermo;
- ricostruzione dopo un resize;
- celle terminali parziali.

### 13.2 Confronto con la scansione brute-force

Mantenere nei test una funzione di riferimento `O(n²)`.

Per diversi stormi deterministici:

1. calcolare l'insieme dei vicini con la scansione completa;
2. calcolarlo con la griglia;
3. ordinare o marcare gli indici;
4. verificare che gli insiemi coincidano per ogni boid.

Scenari:

- distribuzione uniforme;
- cluster molto denso;
- boid sui bordi delle celle;
- boid fuori schermo;
- raggi da 1 a 12 celle;
- raggio che attraversa entrambi i bordi dello schermo.

### 13.3 Test del motore

Con posizioni e direzioni prefissate, confrontare una iterazione del motore
ottimizzato con la versione di riferimento. Usare una tolleranza per i `double`
e verificare:

- direzione;
- posizione `x/y`;
- frame dello sprite;
- numero di vicini.

### 13.4 Test CLI e input

- il default equivale a 3 celle e 36 pixel;
- `P` incrementa di una cella;
- `p` decrementa di una cella;
- i limiti 1 e 12 non vengono superati;
- la documentazione riporta gli stessi valori.

### 13.5 Verifiche tecniche

- build GCC con `-Wall -Wextra -Werror`;
- build Clang;
- test con AddressSanitizer e UndefinedBehaviorSanitizer;
- `clang-format --dry-run --Werror`;
- `git diff --check`;
- smoke test interattivo con avvio e uscita tramite `q`;
- test di backpressure con output saturo e controllo del suffisso non inviato;
- prova manuale in un terminale Kitty compatibile;
- confronto indicativo del tempo per frame prima e dopo la modifica.

## 14. Aggiornamento della documentazione

Il README deve spiegare:

- celle fisse da 12×12 pixel;
- raggio predefinito di 3 celle, equivalente a 36 pixel;
- intervallo da 1 a 12 celle;
- `P` e `p` modificano il raggio di una cella;
- la griglia riduce i candidati ma il filtro finale resta circolare;
- la complessità media migliora, mentre il caso peggiore resta `O(n²)`.

La vecchia costante `PERCEPTION_RADIUS = 35` e il passo da 3 pixel devono essere
rimossi dalla sezione di configurazione.

## 15. Sequenza di implementazione

### Fase 1: configurazione

1. Aggiungere le costanti per dimensione cella e raggio in celle.
2. Modificare `config_t`.
3. Centralizzare il calcolo del raggio in pixel e del quadrato.
4. Aggiornare `P` e `p`.

### Fase 2: struttura della griglia

1. Definire `spatial_grid_t`.
2. Implementare inizializzazione e distruzione.
3. Implementare il resize transazionale.
4. Implementare mapping e clamping delle coordinate.
5. Implementare conteggio, prefix sum e inserimento.

### Fase 3: integrazione nel motore

1. Preparare la griglia dopo l'aggiornamento delle dimensioni dello schermo.
2. Costruirla dallo snapshot a ogni frame.
3. Passarla a `update_birds` e `flock_direction`.
4. Sostituire la scansione completa con la visita delle celle.
5. Conservare il filtro preciso sulla distanza.

### Fase 4: test

1. Aggiungere test deterministici della griglia.
2. Aggiungere il confronto con l'implementazione brute-force.
3. Verificare resize, coordinate esterne e limiti del raggio.
4. Eseguire sanitizzatori, build e smoke test.

### Fase 5: documentazione e misura

1. Aggiornare README e help se necessario.
2. Misurare il numero di candidati e il tempo medio per frame.
3. Verificare manualmente che il comportamento dello stormo non presenti
   regressioni visive.

## 16. Criteri di accettazione

L'implementazione è completa quando:

- la dimensione delle celle è sempre 12×12 pixel;
- il raggio predefinito è 3 celle, cioè 36 pixel;
- `P` e `p` modificano il raggio fra 1 e 12 celle;
- nessuna ricerca normale scansiona direttamente tutti i boid;
- la lista dei vicini coincide con quella brute-force;
- la forma del campo visivo resta circolare;
- snapshot e double buffering mantengono l'aggiornamento simultaneo;
- l'orientamento dello sprite resta sincronizzato;
- resize e coordinate fuori schermo non causano accessi fuori limite;
- non avvengono allocazioni nel percorso ordinario di ogni frame;
- build, test, sanitizzatori e smoke test passano;
- README e codice espongono gli stessi valori.

## 17. Non-obiettivi

Questa modifica non comprende:

- modifica delle formule di separazione, allineamento o coesione;
- introduzione di gruppi o stormi persistenti;
- parallelizzazione multithread;
- SIMD;
- fallback ASCII descritto in `demo.txt`;
- cambiamenti al protocollo Kitty;
- modifica della struttura `bird_t` in Structure of Arrays;
- gestione di ostacoli, predatori o wrapping dei bordi.

Questi interventi potranno essere valutati separatamente dopo avere misurato il
beneficio della griglia uniforme.

## 18. Strategia Git proposta

La modifica può essere consegnata con un commit unico e coerente:

```text
Add spatial grid for boid neighbor queries
```

Il commit deve includere soltanto motore, test, Makefile e documentazione
necessari. `.claude/` e `demo.txt` rimangono fuori salvo una decisione esplicita
separata.
