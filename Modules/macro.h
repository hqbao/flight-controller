#ifndef MACRO_H
#define MACRO_H

#define LIMIT(number, min, max) (number < min ? min : (number > max ? max : number))
#define MAX(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })

#endif
