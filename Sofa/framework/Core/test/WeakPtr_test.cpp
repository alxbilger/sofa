#include <gtest/gtest.h>

#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

using sofa::core::WeakPtr;

template <typename T>
class WeakPtr_Test : public testing::Test
{
public:

    void SetUp() override
    {
        EXPECT_NE(p, nullptr);
    }

    void defaultConstructor()
    {
        WeakPtr<T> ptr;
        EXPECT_EQ(ptr.lock(), nullptr);
    }

    void constructorFromSptr()
    {
        WeakPtr<T> ptr(p);
        EXPECT_NE(ptr.lock(), nullptr);
    }

    void constructorFromWeak()
    {
        WeakPtr<T> ptr(p);
        EXPECT_NE(ptr.lock(), nullptr);

        WeakPtr<T> ptr2(ptr);
        EXPECT_NE(ptr2.lock(), nullptr);
    }

    void operatorEqualFromPointer()
    {
        WeakPtr<T> ptr;

        ptr = p.get();

        EXPECT_NE(ptr.lock(), nullptr);
    }

    void weak()
    {
        WeakPtr<T> ptr(p);

        EXPECT_FALSE(m_isDestroyed);
        p.reset();
        EXPECT_TRUE(m_isDestroyed);

        EXPECT_EQ(ptr.lock(), nullptr);
    }

private:
    bool m_isDestroyed = false;
    core::sptr<T> p = sofa::core::objectmodel::New<T>(&m_isDestroyed);

};


class BaseObjectTest : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(BaseObjectTest, sofa::core::objectmodel::BaseObject);
    BaseObjectTest(bool* isDestroyed = nullptr) : m_isDestroyed(isDestroyed)
    {
        if (m_isDestroyed) *m_isDestroyed = false;
    }
    ~BaseObjectTest() override
    {
        if (m_isDestroyed) *m_isDestroyed = true;
    }

    bool* m_isDestroyed { nullptr };
};


using MyTypes = ::testing::Types<BaseObjectTest>;
TYPED_TEST_SUITE(WeakPtr_Test, MyTypes);

TYPED_TEST(WeakPtr_Test, defaultConstructor)
{
    this->defaultConstructor();
}

TYPED_TEST(WeakPtr_Test, constructorFromSptr)
{
    this->constructorFromSptr();
}

TYPED_TEST(WeakPtr_Test, constructorFromWeak)
{
    this->constructorFromWeak();
}

TYPED_TEST(WeakPtr_Test, operatorEqualFromPointer)
{
    this->constructorFromWeak();
}

TYPED_TEST(WeakPtr_Test, weak)
{
    this->weak();
}

}
