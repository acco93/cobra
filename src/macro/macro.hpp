/**
 * Macros.
 */

#ifndef COBRA_MACRO_HPP
#define COBRA_MACRO_HPP

#define likely(condition) __builtin_expect(static_cast<bool>(condition), 1)
#define unlikely(condition) __builtin_expect(static_cast<bool>(condition), 0)

#endif //COBRA_MACRO_HPP
