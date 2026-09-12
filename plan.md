# Piano di implementazione del pannello dei parametri per Cbirds

Questo documento sostituisce il piano dello spatial hashing, implementato e
rilasciato nel commit `52e4967`.

## 1. Obiettivo

Sostituire la mode line orizzontale in fondo allo schermo con un pannello
verticale in alto a sinistra, in cui ogni parametro regolabile è un cursore
riempito e nessun valore numerico viene stampato. Il cursore **è** la lettura:
quanto è riempita la barra è l'intera affermazione sul valore.

Il pannello:

- si disattiva con `--no-legend`;
- sta ancorato all'angolo in alto a sinistra e non si sposta mai;
- quando è attivo è un **ostacolo invalicabile** per lo stormo, ottenuto con una
  componente di bordo di magnitudine enormemente superiore a ogni altro termine,
  come già si faceva per la virata sul bordo inferiore prima del commit
  `72a18a5`;
- indica per ogni parametro il nome e i due tasti che lo muovono, scritti
  minuscola prima (`b/B`), cioè l'ordine in cui appaiono sulla barra: prima
  quello che abbassa, poi quello che alza.

Non cambia niente del modello Boids: separazione, allineamento, coesione e
virata dai bordi restano quelle che sono, il campo visivo resta circolare, tutti
i boid continuano ad aggiornarsi dallo stesso snapshot.

## 2. Il pannello

Prototipo 19 di `legend.txt`, riempimento a mezza tinta, con i tasti appaiati
del prototipo 17 invertiti in minuscola-maiuscola:

```
╭──────────────────────────╮
│ boundary   ▓▓░░░░░░  b/B │
│ separation ▓░░░░░░░  s/S │
│ cohesion   ▓▓▓▓░░░░  c/C │
│ alignment  ▓▓▓░░░░░  a/A │
│ perception ▓▓▓▓▓░░░  p/P │
│ rate       ▓▓▓░░░░░  r/R │
│                          │
│ quit       q             │
╰──────────────────────────╯
```

Composizione di una riga, che fissa la larghezza:

```
│ + spazio        2
nome                     10   la più lunga è "separation" / "perception"
spazio                    1
barra                     8   LEGEND_BAR_CELLS
due spazi                 2
coppia di tasti           3   "b/B"
spazio + │                2
                    ─────────
                         28
```

Otto righe di contenuto più i due filetti: **28 colonne x 10 righe**, misura
fissa. Non dipende dalla dimensione del terminale, perché dipende solo dal nome
più lungo e dalla larghezza della barra. È una proprietà che il piano sfrutta
più volte: il pannello non cambia mai né posizione né dimensione in celle.

## 3. La barra ha bisogno di un fondo scala

**Questa è la decisione che sblocca tutto il resto.** I quattro pesi hanno oggi
un pavimento ma nessun tetto:

```c
case 'A':
    config.alignment += ALIGNMENT_STEP;   /* nessun limite superiore */
    break;
```

Senza un massimo una barra non ha niente contro cui riempirsi. Si introduce
quindi un tetto per ciascuno, pari a **tre volte il default**:

| parametro | min | default | max proposto | passi di corsa |
|---|---|---|---|---|
| boundary | 0.01 | 0.2 | 0.6 | 30 |
| separation | 0.001 | 0.005 | 0.015 | 14 |
| cohesion | 0.002 | 0.01 | 0.03 | 14 |
| alignment | 0.1 | 1.5 | 4.5 | 44 |
| perception | 1 cella | 3 celle | 5 celle | 4 |
| rate | 30 | 60 | 120 | 18 |

Tre volte il default mette il valore di partenza a circa un terzo della corsa,
che è esattamente come si presenta il pannello nel mockup, e tiene il numero di
pressioni per cella basso abbastanza perché premere un tasto muova la barra in
modo visibile: da 1.75 pressioni per cella (separation, cohesion) a 5.5
(alignment).

Riempimento in celle, frazione calcolata sulla corsa e non sul valore assoluto,
così la barra si vuota completamente sul pavimento:

```c
static int bar_cells(double value, double minimum, double maximum) {
    double fraction = (value - minimum) / (maximum - minimum);
    if (fraction < 0) fraction = 0;
    if (fraction > 1) fraction = 1;
    return (int)(fraction * LEGEND_BAR_CELLS + 0.5);
}
```

`B`, `S`, `C` e `A` vengono clampati al nuovo tetto esattamente come `R` e `P`
già lo sono, il che chiude anche l'asimmetria per cui metà dei controlli era
limitata da un lato solo. **È l'unica modifica del piano che cambia il
comportamento del programma al di fuori del pannello**, ed è il punto su cui
voglio conferma prima di procedere: se preferisci tetti diversi, si cambiano
quattro costanti.

## 4. Geometria e ingombro

```c
enum {
    LEGEND_COLUMNS = 28,
    LEGEND_ROWS = 10,
    LEGEND_BAR_CELLS = 8,
    LEGEND_MIN_COLS = 40,   /* 28 + un corridoio a destra */
    LEGEND_MIN_ROWS = 14    /* 10 + un corridoio sotto */
};
```

L'estensione in pixel si ricava dalla cella del terminale:

```c
screen.legend_width = LEGEND_COLUMNS * screen.cell_width;
screen.legend_height = LEGEND_ROWS * screen.cell_height;
```

A celle 8x16 sono 224x160 pixel.

Da notare che l'area di volo **non** diventa un rettangolo più piccolo: il
pannello occupa un angolo, quindi lo spazio restante è una L. Lo stormo
conserva tutta la larghezza sotto il pannello e tutta l'altezza a destra. Per
questo la condizione di disattivazione non guarda l'area totale ma i due
corridoi: servono almeno 12 colonne a destra e 4 righe sotto, da cui i 40x14 di
soglia. Sono numeri di partenza, non misurati.

Il pannello scompare quando:

- è stato passato `--no-legend`;
- il terminale è sotto 40 colonne o sotto 14 righe.

In entrambi i casi lo stormo riprende l'intero viewport e nessuna forza agisce.

## 5. Il pannello come ostacolo invalicabile

### 5.1 Quali posizioni sono proibite

Uno sprite viene disegnato a partire dal suo angolo superiore sinistro e occupa
`[x, x + bird_size] x [y, y + bird_size]`. Si sovrappone al pannello, che è
`[0, legend_width] x [0, legend_height]`, esattamente quando:

```
x < legend_width  &&  y < legend_height
```

Non serve allargare il rettangolo a destra o in basso: lo sprite si estende in
quelle direzioni, quindi la sovrapposizione dipende solo dal fatto che l'angolo
sia dentro. Questo rettangolo è il **rettangolo proibito**: nessuno sprite deve
mai averci l'angolo dentro, ed è ciò che i test verificano.

### 5.2 Dove agisce la forza

Se la forza agisse solo dentro il rettangolo proibito, un boid ci entrerebbe per
un frame prima di essere respinto, e per quel frame lo si vedrebbe sopra il
pannello. La forza agisce quindi su un **rettangolo di virata**, il pannello
ingrandito di una distanza pari a un frame di volo:

```
x < legend_width + speed  &&  y < legend_height + speed
```

Con questo margine la non-sovrapposizione si dimostra. Un boid appena fuori dal
rettangolo di virata sta a `x = legend_width + speed + ε`; nel frame in cui vi
entra la sua posizione peggiore è `legend_width + ε`, cioè ancora fuori dal
rettangolo proibito; a quel punto la forza è attiva e il frame successivo lo
porta a `legend_width + ε + speed`. Il rettangolo proibito non è raggiungibile.
`speed` è già derivata dal frame rate, quindi il margine segue automaticamente
`R` e `r`.

### 5.3 Direzione della spinta

Verso l'uscita più vicina, misurata sul rettangolo di virata:

```c
double escape_x = screen.legend_width + config.speed - bird->x;
double escape_y = screen.legend_height + config.speed - bird->y;
if (escape_x <= escape_y)
    boundary.x = LEGEND_PUSH;   /* fuori a destra */
else
    boundary.y = LEGEND_PUSH;   /* fuori in basso */
```

Il pannello è in un angolo, quindi le uscite sono solo destra e basso: la
spinta non punta mai verso un bordo dello schermo, e non entra in conflitto con
le bande di virata del bordo superiore e sinistro, che spingono nelle stesse due
direzioni.

`LEGEND_PUSH = 100000`, la stessa scala della vecchia virata sul bordo
inferiore, scelta perché domini ogni altro termine. Nel ramo con vicini la
componente entra nella somma pesata come `100000 * config.boundary`, che a
`boundary` sul pavimento (0.01) vale ancora 1000 contro termini dell'ordine
dell'unità. Nel ramo senza vicini entra come `cos(direzione) + 100000 *
config.boundary`. In entrambi i casi la direzione risultante è l'asse di fuga.

### 5.4 Dove va nel codice

Tutto dentro `boundary_vector`, che è già la funzione che restituisce la
componente di bordo, è già condivisa dal motore e dal riferimento brute-force
nei test, e viene già chiamata una volta per boid per frame. Il costo aggiunto
sono due confronti per boid.

```c
static vector_t boundary_vector(const bird_t *bird) {
    vector_t boundary = {0, 0};
    if (legend_repels(bird, &boundary)) return boundary;
    /* ... bande di schermo attuali, invariate ... */
}
```

Uscire subito quando il pannello respinge tiene le due logiche separate e rende
evidente che la spinta del pannello non si somma a niente.

## 6. Posizionamento iniziale

`initialize_birds` campiona già la regione non coperta dalle bande di virata.
Quella regione può intersecare il rettangolo di virata del pannello: a 640x384,
con `turn_x = 213` e `turn_y = 128`, il pannello arriva a 224x160 pixel e
l'angolo si sovrappone. Si aggiunge un rifiuto:

```c
for (int attempt = 0; attempt < SPAWN_ATTEMPTS; attempt++) {
    /* campiona x, y */
    if (!legend_turn_zone(x, y)) break;
}
/* dopo SPAWN_ATTEMPTS, ripiega subito sotto il pannello */
```

Il numero di tentativi è limitato e c'è un ripiego deterministico, così la
funzione non può girare a vuoto su un terminale dove la regione libera è quasi
tutta coperta.

## 7. Disegno

Il pannello è testo, dieci righe, scritte con la `kitty_graphics_write_text` che
esiste già: condivide il buffer con i comandi grafici, quindi viaggia nello
stesso update sincronizzato e nello stesso flush con controllo di flusso.

```
sync begin -> a=d,d=a -> placement -> dieci righe del pannello -> sync end
```

Il pannello resta l'ultima cosa disegnata, sopra lo stormo. Dato che i boid non
possono entrarci non è strettamente necessario, ma è gratis ed è una difesa in
più.

Costo: dieci righe da circa 40 byte, ossia circa 400 byte per frame contro i
29 KB che un frame da 800 boid già spende. Ridisegnare sempre costa l'1,4% e
non vale un dirty flag.

**Nessun erase di schermo, mai.** È la regola che ha già rotto lo stormo una
volta: `ESC[2J` a sprite caricati li cancella, e ogni `a=p` successivo punta a
un'immagine che non esiste più. Il pannello è ancorato a (0,0) e di dimensione
costante in celle, quindi non lascia mai testo orfano da nessuna parte: l'unico
caso in cui va cancellato qualcosa è quando il pannello si **spegne**, perché il
terminale è scesso sotto soglia. Si tiene un flag `legend_drawn`, e nel frame in
cui passa da disegnato a non disegnato si emettono dieci `CSI K`, una per riga.

## 8. Cosa viene rimosso

La mode line in fondo esce interamente:

- `reserve_legend_row`, e con essa la riduzione di `screen.rows` e
  `screen.height`: lo stormo si riprende la riga in fondo e i `bird_size` pixel
  che cedeva;
- `build_legend`, `queue_legend` nella forma attuale, `weights_are_default` e il
  sigillo `-:---` / `-:**-`, che le barre rendono superfluo;
- le tre fasce di larghezza `LEGEND_NARROW_COLS`, `LEGEND_MEDIUM_COLS`,
  `LEGEND_WIDE_COLS` e i buffer `LEGEND_TEXT_MAX` / `LEGEND_LINE_MAX`;
- `screen.legend_row` e `drawn_legend_row`, sostituiti da `screen.legend_width`,
  `screen.legend_height` e `legend_drawn`;
- i test `test_legend_fits_every_width`,
  `test_legend_sigil_tracks_the_weights`, `test_legend_row_is_reserved`,
  `test_legend_absent_on_a_tiny_viewport` e
  `test_frame_carries_the_legend`.

`screen.turn_bottom` **resta**: è la banda di virata del bordo inferiore, non
ha niente a che vedere con la legenda.

## 9. Riga di comando

`read_options` accetta oggi solo opzioni di due caratteri più `-h` / `--help`, e
rifiuta tutto il resto. Si aggiunge `--no-legend` come caso esplicito prima del
controllo di lunghezza, e una riga in `usage`. È un interruttore di avvio: non
esiste un tasto per riaccendere il pannello a runtime, e il piano non ne
introduce uno.

## 10. Casi limite

| caso | comportamento |
|---|---|
| terminale sotto 40x14 | pannello spento, viewport intero allo stormo, dieci `CSI K` nel frame di spegnimento |
| terminale che torna sopra soglia | pannello riacceso, si ridisegna da solo il frame dopo |
| resize che cambia la cella | `legend_width` e `legend_height` cambiano, i boid rimasti dentro vengono spinti fuori dalla forza |
| `-s 64` | il margine di virata non dipende da `bird_size`, ma il rettangolo proibito è definito sull'angolo dello sprite: resta corretto |
| `R` / `r` | `speed` cambia, il margine di virata la segue, il margine resta esattamente un frame |
| boid senza vicini dentro il rettangolo | ramo senza vicini, la spinta domina comunque |
| `--no-legend` | nessuna forza, nessun disegno, nessuna riserva |
| pannello più grande del viewport | impossibile: sotto soglia si spegne |

## 11. Strategia di test

### 11.1 Contenuto del pannello

- dieci righe, ognuna esattamente 28 celle;
- angoli tondi, filetti chiusi;
- sei righe di parametro, ognuna con nome, barra da 8 celle e coppia di tasti;
- **la coppia è minuscola prima**: `b/B` presente, `B/b` assente. È la richiesta
  esplicita e va asserita come tale;
- riga `quit q`.

### 11.2 Mappatura della barra

- valore sul pavimento, zero celle piene;
- valore sul tetto, otto celle piene;
- default, il numero di celle del mockup;
- monotona: salire di un passo non può diminuire il riempimento;
- una pressione di `S` o `C` cambia il riempimento, che è il motivo per cui i
  tetti sono quelli.

### 11.3 I tetti

- `B`, `S`, `C`, `A` si fermano al massimo e non lo superano;
- il pavimento continua a funzionare come prima.

### 11.4 La forza

- dentro il rettangolo di virata la direzione restituita è l'asse di fuga,
  a meno di 1e-9;
- vicino al bordo destro spinge a destra, vicino al bordo inferiore spinge in
  basso, in fondo all'angolo spinge lungo la fuga più corta;
- **domina i vicini**: un boid dentro il rettangolo con cinquanta vicini che lo
  tirano nella direzione opposta esce comunque;
- fuori dal rettangolo di virata la funzione restituisce esattamente quello che
  restituiva prima, il che tiene onesti i test delle bande di schermo.

### 11.5 Non attraversabilità

Il test che conta. Da una griglia di posizioni appena fuori dal rettangolo di
virata, con direzioni su tutto il giro, si simulano N frame e si asserisce che
**nessun boid finisca mai con l'angolo dentro il rettangolo proibito**. Da
ripetere ai frame rate estremi, 30 e 120, perché cambiano `speed` e quindi il
margine.

### 11.6 Spawn

Nessun boid parte dentro il rettangolo di virata, su terminali di dimensioni
diverse, incluso uno dove la regione libera è quasi tutta coperta.

### 11.7 Frame

- il pannello sta dentro l'update sincronizzato, prima del sync end;
- le dieci righe sono indirizzate alle righe 1..10 sul filo;
- **nessun frame contiene `ESC[2J` o `ESC[3J`**: il test esiste già, va tenuto;
- passando sotto soglia si emettono dieci `CSI K` una volta sola, e non a ogni
  frame successivo;
- con `--no-legend` non si emette né pannello né erase.

### 11.8 Equivalenza griglia e brute force

Il test esistente confronta il motore con la scansione lineare, e le due strade
condividono `boundary_vector`, quindi l'equivalenza vale per costruzione. Va
comunque rieseguito con il pannello attivo, e con posizioni che cadono dentro il
rettangolo di virata, per essere sicuri che la nuova uscita anticipata non
rompa quella condivisione.

### 11.9 Verifica su pty

I test controllano i byte emessi, non ciò che il terminale ne fa: è esattamente
il buco da cui è passato il bug dell'erase. Come per quel fix, si verifica su un
pty reale che in una corsa completa ci siano 90 upload, zero erase di schermo,
il pannello a ogni frame, e che il numero di placement non crolli.

## 12. Documentazione

- `README.md`: la sezione della mode line viene sostituita dal pannello, con il
  blocco disegnato, `--no-legend` fra le opzioni, i tetti nella tabella dei
  default, e una riga sul fatto che il pannello è un ostacolo per lo stormo;
- la nota che `demo.gif` precede il pannello resta valida;
- `legend.txt` non è tracciato e non entra in repo: da decidere se vada in
  `.gitignore` o cancellato una volta scelto il prototipo.

## 13. Sequenza di implementazione

1. **Tetti dei pesi.** Indipendente da tutto il resto, e sblocca la scala della
   barra. Commit a sé: cambia il comportamento dei tasti.
2. **Geometria e `--no-legend`.** Costanti, campi di `screen`, parsing, soglie.
   Ancora nessun disegno e nessuna forza.
3. **La forza e lo spawn.** Con il test di non attraversabilità, che è il test
   che dice se il piano funziona.
4. **Il pannello.** Costruzione delle dieci righe, disegno nel frame, erase allo
   spegnimento, e rimozione della mode line in fondo.
5. **README e verifica su pty.**

Ogni passo lascia i test verdi, il binario funzionante, e nessun warning con
`-Wall -Wextra -Wpedantic -Wshadow -Wconversion`.

## 14. Criteri di accettazione

- il pannello appare in alto a sinistra, 28x10, come il blocco della sezione 2,
  con i tasti scritti `b/B`;
- le barre si muovono premendo i tasti, e nessun numero compare nel pannello;
- nessuno sprite si sovrappone mai al pannello, verificato dal test di non
  attraversabilità a 30 e 120 FPS;
- `--no-legend` restituisce il comportamento attuale senza pannello, senza
  forza e con il viewport intero;
- sotto 40x14 il pannello si spegne da sé e non lascia testo sullo schermo;
- nessun frame emette un erase di schermo;
- `make test` verde, pulito sotto ASan e UBSan, `clang-format` conforme;
- una corsa su pty mostra placement stabili e zero erase.

## 15. Non obiettivi

- un tasto per mostrare e nascondere il pannello a runtime;
- pannello spostabile o ancorabile ad altri angoli;
- pannello ridimensionabile o a più fasce di larghezza, come faceva la mode
  line: la misura è fissa, e sotto soglia si spegne invece di comprimersi;
- valori numerici, in qualunque forma;
- il sigillo Emacs di buffer modificato, che le barre rendono inutile;
- ostacoli generici nel campo: la forza è scritta per un rettangolo ancorato
  all'angolo in alto a sinistra, non per un ostacolo qualsiasi.

## 16. Decisioni aperte

1. **I tetti della sezione 3.** Tre volte il default è una proposta. È l'unica
   cosa che cambia il comportamento fuori dal pannello, e cambiarla costa
   quattro costanti.
2. **Le soglie 40x14.** Scelte per lasciare 12 colonne e 4 righe di corridoio,
   non misurate su terminali reali.
3. **`quit q` dentro il pannello.** Il prototipo 19 lo tiene nel corpo, dopo una
   riga vuota. L'alternativa, che nei prototipi era il 47, è metterlo nel filetto
   inferiore (`╰─ q quit ───╯`) e guadagnare due righe, lasciando nel corpo solo
   cursori. Il piano segue il 19 come richiesto.
