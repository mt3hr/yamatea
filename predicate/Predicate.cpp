#include "Predicate.h"

Predicate::~Predicate()
{
}

// オーバーライドして使って
bool Predicate::test()
{
    return false;
}