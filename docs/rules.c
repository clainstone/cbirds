/* The three rules, from boids.c */
for (int slot = grid->offsets[cell];
     slot < grid->offsets[cell + 1]; slot++) {
    int i = grid->indices[slot];
    const bird_t *other = &birds[i];
    double dx = target->x - other->x;
    double dy = target->y - other->y;
    separation.x += dx;
    separation.y += dy;
    trig_entry_t heading =
        trig_lookup(other->direction);
    alignment.x += heading.cosine;
    alignment.y += heading.sine;
    cohesion.x += other->x;
    cohesion.y += other->y;
}
