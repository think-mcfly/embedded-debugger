#include <unity.h>

void test_function_calculator_division(void)
{
    TEST_ASSERT_EQUAL(2, 4 / 2);
    TEST_ASSERT_EQUAL(3, 6 / 2);
    TEST_ASSERT_EQUAL(4, 8 / 2);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_function_calculator_division);
    return UNITY_END();
}