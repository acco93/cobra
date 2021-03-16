/**
 * Non-copyable mixin; see more here: https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Non-copyable_Mixin
 */

#ifndef COBRA_NONCOPYABLE_HPP
#define COBRA_NONCOPYABLE_HPP

template <class T>
class NonCopyable
{
public:
    NonCopyable (const NonCopyable &) = delete;
    T & operator = (const T &) = delete;

protected:
    NonCopyable () = default;
    ~NonCopyable () = default; /// Protected non-virtual destructor
};


#endif //COBRA_NONCOPYABLE_HPP
