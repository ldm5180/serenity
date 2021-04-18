/*
 * Copyright (c) 2021, Idan Horowitz <idan.horowitz@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <AK/Array.h>
#include <AK/Concepts.h>

namespace AK {

template<Integral K>
class BaseRedBlackTree {
public:
    [[nodiscard]] size_t size() const { return m_size; }
    [[nodiscard]] bool is_empty() const { return m_size == 0; }

    enum class Color : bool {
        Red,
        Black,
    };

    enum class NodeIndex : int {
        Left = 0,
        Right = 1,
        Parent = 2,
    };

    struct Node {
        Array<Node*, 3> nodes {};

        Color color { Color::Red };

        K key;

        explicit Node(K key)
            : key(key)
        {
        }

        auto& left_child() { return nodes[int(NodeIndex::Left)]; }
        auto& right_child() { return nodes[int(NodeIndex::Right)]; }
        auto& parent() { return nodes[int(NodeIndex::Parent)]; }
    };

protected:
    BaseRedBlackTree() = default; // These are protected to ensure no one instantiates the leaky base red black tree directly

    template<auto TSide>
    constexpr void rotate(Node* subtree_root)
    {
        constexpr auto side = int(TSide);
        constexpr auto other_side = TSide == NodeIndex::Left ? int(NodeIndex::Right) : int(NodeIndex::Left);
        VERIFY(subtree_root);
        auto* pivot = subtree_root->nodes[other_side];
        VERIFY(pivot);
        auto* parent = subtree_root->parent();

        update_child<NodeIndex(other_side)>(subtree_root, pivot->nodes[side]);
        update_pivot_child<TSide>(pivot, subtree_root);
        update_pivot_parent(pivot, parent, subtree_root);
    }

    static Node* find(Node* node, K key)
    {
        while (node && node->key != key) {
            if (key < node->key) {
                node = node->left_child();
            } else {
                node = node->right_child();
            }
        }
        return node;
    }

    static Node* find_largest_not_above(Node* node, K key)
    {
        Node* candidate = nullptr;
        while (node) {
            if (key == node->key) {
                return node;
            } else if (key < node->key) {
                node = node->left_child();
            } else {
                candidate = node;
                node = node->right_child();
            }
        }
        return candidate;
    }

    void insert(Node* node)
    {
        VERIFY(node);
        Node* parent = nullptr;
        Node* temp = m_root;
        while (temp) {
            parent = temp;
            if (node->key < temp->key) {
                temp = temp->left_child();
            } else {
                temp = temp->right_child();
            }
        }
        if (!parent) { // new root
            node->color = Color::Black;
            m_root = node;
            m_size = 1;
            m_minimum = node;
            return;
        } else if (node->key < parent->key) { // we are the left child
            parent->left_child() = node;
        } else { // we are the right child
            parent->right_child() = node;
        }
        node->parent() = parent;

        if (node->parent()->parent()) // no fixups to be done for a height <= 2 tree
            insert_fixups(node);

        m_size++;
        if (m_minimum->left_child() == node)
            m_minimum = node;
    }

    void insert_fixups(Node* node)
    {
        VERIFY(node && node->color == Color::Red);
        while (node->parent() && node->parent()->color == Color::Red) {
            auto* grand_parent = node->parent()->parent();
            if (grand_parent->right_child() == node->parent()) {
                color_fixup<NodeIndex::Left>(node, grand_parent);
            } else {
                color_fixup<NodeIndex::Right>(node, grand_parent);
            }
        }
        m_root->color = Color::Black; // the root should always be black
    }

    void remove(Node* node)
    {
        VERIFY(node);

        // special case: deleting the only node
        if (m_size == 1) {
            m_root = nullptr;
            m_size = 0;
            return;
        }

        if (m_minimum == node)
            m_minimum = successor(node);

        // removal assumes the node has 0 or 1 child, so if we have 2, relink with the successor first (by definition the successor has no left child)
        // FIXME: since we dont know how a value is represented in the node, we can't simply swap the values and keys, and instead we relink the nodes
        //  in place, this is quite a bit more expensive, as well as much less readable, is there a better way?
        if (node->left_child() && node->right_child()) {
            auto* successor_node = successor(node); // this is always non-null as all nodes besides the maximum node have a successor, and the maximum node has no right child
            auto neighbour_swap = successor_node->parent() == node;
            node->left_child()->parent() = successor_node;
            if (!neighbour_swap)
                node->right_child()->parent() = successor_node;
            if (node->parent()) {
                if (node->parent()->left_child() == node) {
                    node->parent()->left_child() = successor_node;
                } else {
                    node->parent()->right_child() = successor_node;
                }
            } else {
                m_root = successor_node;
            }
            if (successor_node->right_child())
                successor_node->right_child()->parent() = node;
            if (neighbour_swap) {
                successor_node->parent() = node->parent();
                node->parent() = successor_node;
            } else {
                if (successor_node->parent()) {
                    if (successor_node->parent()->left_child() == successor_node) {
                        successor_node->parent()->left_child() = node;
                    } else {
                        successor_node->parent()->right_child() = node;
                    }
                } else {
                    m_root = node;
                }
                swap(node->parent(), successor_node->parent());
            }
            swap(node->left_child(), successor_node->left_child());
            if (neighbour_swap) {
                node->right_child() = successor_node->right_child();
                successor_node->right_child() = node;
            } else {
                swap(node->right_child(), successor_node->right_child());
            }
            swap(node->color, successor_node->color);
        }

        auto* child = node->left_child() ?: node->right_child();

        if (child)
            child->parent() = node->parent();
        if (node->parent()) {
            if (node->parent()->left_child() == node)
                node->parent()->left_child() = child;
            else
                node->parent()->right_child() = child;
        } else {
            m_root = child;
        }

        // if the node is red then child must be black, and just replacing the node with its child should result in a valid tree (no change to black height)
        if (node->color != Color::Red)
            remove_fixups(child, node->parent());

        m_size--;
    }

    // We maintain parent as a separate argument since node might be null
    void remove_fixups(Node* node, Node* parent)
    {
        while (node != m_root && (!node || node->color == Color::Black)) {
            if (parent->left_child() == node) {
                auto* sibling = parent->right_child();
                if (sibling->color == Color::Red) {
                    sibling->color = Color::Black;
                    parent->color = Color::Red;
                    rotate<NodeIndex::Left>(parent);
                    sibling = parent->right_child();
                }
                if ((!sibling->left_child() || sibling->left_child()->color == Color::Black) && (!sibling->right_child() || sibling->right_child()->color == Color::Black)) {
                    sibling->color = Color::Red;
                    node = parent;
                } else {
                    if (!sibling->right_child() || sibling->right_child()->color == Color::Black) {
                        sibling->left_child()->color = Color::Black; // null check?
                        sibling->color = Color::Red;
                        rotate<NodeIndex::Right>(sibling);
                        sibling = parent->right_child();
                    }
                    sibling->color = parent->color;
                    parent->color = Color::Black;
                    sibling->right_child()->color = Color::Black; // null check?
                    rotate<NodeIndex::Left>(parent);
                    node = m_root; // fixed
                }
            } else {
                auto* sibling = parent->left_child();
                if (sibling->color == Color::Red) {
                    sibling->color = Color::Black;
                    parent->color = Color::Red;
                    rotate<NodeIndex::Right>(parent);
                    sibling = parent->left_child();
                }
                if ((!sibling->left_child() || sibling->left_child()->color == Color::Black) && (!sibling->right_child() || sibling->right_child()->color == Color::Black)) {
                    sibling->color = Color::Red;
                    node = parent;
                } else {
                    if (!sibling->left_child() || sibling->left_child()->color == Color::Black) {
                        sibling->right_child()->color = Color::Black; // null check?
                        sibling->color = Color::Red;
                        rotate<NodeIndex::Left>(sibling);
                        sibling = parent->left_child();
                    }
                    sibling->color = parent->color;
                    parent->color = Color::Black;
                    sibling->left_child()->color = Color::Black; // null check?
                    rotate<NodeIndex::Right>(parent);
                    node = m_root; // fixed
                }
            }
            parent = node->parent();
        }
        node->color = Color::Black; // by this point node can't be null
    }

    static Node* successor(Node* node)
    {
        return next<NodeIndex::Right, NodeIndex::Left>(node);
    }

    static Node* predecessor(Node* node)
    {
        return next<NodeIndex::Left, NodeIndex::Right>(node);
    }

    template<auto TDirection, auto TOtherDirection>
    static Node* next(Node* node)
    {
        constexpr auto direction = int(TDirection);
        constexpr auto other_direction = int(TOtherDirection);
        VERIFY(node);
        if (node->nodes[direction]) {
            node = node->nodes[direction];
            while (node->nodes[other_direction])
                node = node->nodes[other_direction];
            return node;
        } else {
            auto temp = node->parent();
            while (temp && node == temp->nodes[direction]) {
                node = temp;
                temp = temp->parent();
            }
            return temp;
        }
    }

    Node* m_root { nullptr };
    size_t m_size { 0 };
    Node* m_minimum { nullptr }; // maintained for O(1) begin()

private:
    template<auto TSide>
    constexpr void update_child(auto* subtree_root, const auto& pivot_other_child)
    {
        constexpr auto side = int(TSide);
        subtree_root->nodes[side] = pivot_other_child;
        if (subtree_root->nodes[side])
            subtree_root->nodes[side]->parent() = subtree_root;
    }

    template<auto TSide>
    constexpr void update_pivot_child(auto* pivot, auto* subtree_root)
    {
        constexpr auto side = int(TSide);
        pivot->nodes[side] = subtree_root;
        subtree_root->parent() = pivot;
    }

    void update_pivot_parent(auto* pivot, auto* parent, const auto* subtree_root)
    {
        pivot->parent() = parent;
        if (!parent) { // new root
            m_root = pivot;
        } else if (parent->left_child() == subtree_root) { // we are the left child
            parent->left_child() = pivot;
        } else { // we are the right child
            parent->right_child() = pivot;
        }
    }

    template<auto TSide>
    constexpr void color_fixup(auto& node, auto* grand_parent)
    {
        constexpr auto side = int(TSide);
        auto* uncle = grand_parent->nodes[side];
        if (uncle && uncle->color == Color::Red) {
            node->parent()->color = Color::Black;
            uncle->color = Color::Black;
            grand_parent->color = Color::Red;
            node = grand_parent;
        } else {
            if (node->parent()->nodes[side] == node) {
                node = node->parent();
                if constexpr (TSide == NodeIndex::Left) {
                    rotate<NodeIndex::Right>(node);
                } else {
                    rotate<NodeIndex::Left>(node);
                }
            }
            node->parent()->color = Color::Black;
            grand_parent->color = Color::Red;
            if constexpr (TSide == NodeIndex::Right) {
                rotate<NodeIndex::Right>(grand_parent);
            } else {
                rotate<NodeIndex::Left>(grand_parent);
            }
        }
    }
};

template<typename TreeType, typename ElementType>
class RedBlackTreeIterator {
public:
    RedBlackTreeIterator() = default;
    bool operator!=(const RedBlackTreeIterator& other) const { return m_node != other.m_node; }
    RedBlackTreeIterator& operator++()
    {
        if (!m_node)
            return *this;
        m_prev = m_node;
        // the complexity is O(logn) for each successor call, but the total complexity for all elements comes out to O(n), meaning the amortized cost for a single call is O(1)
        m_node = static_cast<typename TreeType::Node*>(TreeType::successor(m_node));
        return *this;
    }
    RedBlackTreeIterator& operator--()
    {
        if (!m_prev)
            return *this;
        m_node = m_prev;
        m_prev = static_cast<typename TreeType::Node*>(TreeType::predecessor(m_prev));
        return *this;
    }
    ElementType& operator*() { return m_node->value; }
    ElementType* operator->() { return &m_node->value; }
    [[nodiscard]] bool is_end() const { return !m_node; }
    [[nodiscard]] bool is_begin() const { return !m_prev; }

private:
    friend TreeType;
    explicit RedBlackTreeIterator(typename TreeType::Node* node, typename TreeType::Node* prev = nullptr)
        : m_node(node)
        , m_prev(prev)
    {
    }
    typename TreeType::Node* m_node { nullptr };
    typename TreeType::Node* m_prev { nullptr };
};

template<Integral K, typename V>
class RedBlackTree : public BaseRedBlackTree<K> {
public:
    RedBlackTree() = default;
    ~RedBlackTree()
    {
        clear();
    }

    using BaseTree = BaseRedBlackTree<K>;

    V* find(K key)
    {
        auto* node = static_cast<Node*>(BaseTree::find(this->m_root, key));
        if (!node)
            return nullptr;
        return &node->value;
    }

    V* find_largest_not_above(K key)
    {
        auto* node = static_cast<Node*>(BaseTree::find_largest_not_above(this->m_root, key));
        if (!node)
            return nullptr;
        return &node->value;
    }

    void insert(K key, const V& value)
    {
        insert(key, V(value));
    }

    void insert(K key, V&& value)
    {
        auto* node = new Node(key, move(value));
        BaseTree::insert(node);
    }

    using Iterator = RedBlackTreeIterator<RedBlackTree, V>;
    friend Iterator;
    Iterator begin() { return Iterator(static_cast<Node*>(this->m_minimum)); }
    Iterator end() { return {}; }
    Iterator begin_from(K key) { return Iterator(static_cast<Node*>(BaseTree::find(this->m_root, key))); }

    using ConstIterator = RedBlackTreeIterator<const RedBlackTree, const V>;
    friend ConstIterator;
    ConstIterator begin() const { return ConstIterator(static_cast<Node*>(this->m_minimum)); }
    ConstIterator end() const { return {}; }
    ConstIterator begin_from(K key) const { return ConstIterator(static_cast<Node*>(BaseTree::find(this->m_root, key))); }

    V unsafe_remove(K key)
    {
        auto* node = BaseTree::find(this->m_root, key);
        VERIFY(node);

        BaseTree::remove(node);

        V temp = move(static_cast<Node*>(node)->value);

        node->right_child() = nullptr;
        node->left_child() = nullptr;
        delete node;

        return temp;
    }

    bool remove(K key)
    {
        auto* node = BaseTree::find(this->m_root, key);
        if (!node)
            return false;

        BaseTree::remove(node);

        node->right_child() = nullptr;
        node->left_child() = nullptr;
        delete node;

        return true;
    }

    void clear()
    {
        if (this->m_root) {
            delete this->m_root;
            this->m_root = nullptr;
        }
        this->m_minimum = nullptr;
        this->m_size = 0;
    }

private:
    struct Node : BaseRedBlackTree<K>::Node {

        V value;

        Node(K key, V value)
            : BaseRedBlackTree<K>::Node(key)
            , value(move(value))
        {
        }

        ~Node()
        {
            if (this->left_child())
                delete this->left_child();
            if (this->right_child())
                delete this->right_child();
        }
    };
};

}

using AK::RedBlackTree;
