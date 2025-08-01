#pragma once

namespace data_structure
{
    class node
    {
    public:
        node() = default;
        node(node* prev, node* next)
            : m_prev(prev)
            , m_next(next)
        {}

        void insert(node* where)
        {
            m_prev = where;
            m_next = where->m_next;
            where->m_next->m_prev = this;
            where->m_next = this;
        }

        void remove()
        {
            if(m_next==nullptr && m_prev==nullptr)
                return;

            auto prev = m_prev;
            auto next = m_next;
            prev->m_next = next;
            next->m_prev = prev;

            m_next = nullptr;
            m_prev = nullptr;
        }

        node* prev() const { return m_prev; }
        node* next() const { return m_next; }

    private:
        node* m_prev = nullptr;
        node* m_next = nullptr;
    };

}