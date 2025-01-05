#pragma once

#include <stdint.h>
#include <stddef.h>

namespace mip
{
    class Id;

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Represents an index ranging from 0..N excluding N.
    ///
    /// Use this to help avoid off-by-one errors when using indices into arrays at
    /// the same time as referring to MIP values which typically start at 1.
    ///
    /// Use the .index() or .id() methods to get the actual value, depending on
    /// what you need. The index() method returns an unsigned int in the range [0,N-1]
    /// while the id() method returns an unsigned int in the range [1,N]
    ///
    /// isAssigned() can be used to see if the value is valid, i.e. initialized.
    /// isValid(N) can be used to check if the value is within a maximum count.
    ///
    /// The default value with no initialization is INVALID.
    ///
    /// This is interchangeable with the Id class below.
    ///
    class Index
    {
    private:
        unsigned int INVALID = -1;

    public:
        constexpr explicit Index(unsigned int index) : m_index(index) {}
        constexpr Index() : m_index(INVALID) {}

        constexpr Index& setFromIndex(unsigned int index) { m_index = index; return *this; }
        constexpr Index& setFromId(unsigned int id) { m_index = id-1; return *this; }

        constexpr unsigned int index() const { return m_index; }
        constexpr unsigned int id() const { return m_index+1; }

        constexpr bool operator==(const Index& other) const { return m_index == other.m_index; }
        constexpr bool operator!=(const Index& other) const { return m_index != other.m_index; }

        constexpr bool isAssigned() const { return m_index != INVALID; }
        constexpr bool isValid(size_t max_count) const { return m_index < max_count; }

        constexpr void clear() { m_index = INVALID; }

        Index& operator++() { m_index++; return *this; }
        Index& operator--() { m_index--; return *this; }
        Index operator++(int) { Index tmp(*this); m_index++; return tmp; }
        Index operator--(int) { Index tmp(*this); m_index--; return tmp; }

        constexpr Index(const Id& other);
        constexpr Index& operator=(const Id& other);

    private:
        unsigned int m_index;
    };

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Representes an ID number ranging from 1..N including N.
    ///
    /// This is interchangeable with the Index class above.
    ///
    class Id
    {
    private:
        unsigned int INVALID = 0;

    public:
        constexpr explicit Id(unsigned int id) : m_id(id) {}
        constexpr Id() : m_id(INVALID) {}

        constexpr Id& setFromIndex(unsigned int index) { m_id = index+1; return *this; }
        constexpr Id& setFromId(unsigned int id) { m_id = id; return *this; }

        constexpr unsigned int index() const { return m_id-1; }
        constexpr unsigned int id() const { return m_id; }

        constexpr bool operator==(const Id& other) const { return m_id == other.m_id; }
        constexpr bool operator!=(const Id& other) const { return m_id != other.m_id; }

        constexpr bool isAssigned() const { return m_id != INVALID; }
        constexpr bool isValid(size_t max_count) const { return index() < max_count; }

        constexpr void clear() { m_id = INVALID; }

        Id& operator++() { m_id++; return *this; }
        Id& operator--() { m_id--; return *this; }
        Id operator++(int) { Id tmp(*this); m_id++; return tmp; }
        Id operator--(int) { Id tmp(*this); m_id--; return tmp; }

        constexpr Id(const Index& other) : m_id(other.id()) {}
        constexpr Id& operator=(const Index& other) { return setFromIndex(other.index()); }

    private:
        unsigned int m_id;
    };

    inline constexpr Index::Index(const Id& other) : m_index(other.index()) {}
    inline constexpr Index& Index::operator=(const Id& other) { return setFromIndex(other.index()); }

    inline constexpr bool operator==(Id l, Index r) { return l.index() == r.index(); }
    inline constexpr bool operator==(Index l, Id r) { return l.index() == r.index(); }

    // Get a pointer to a value in the container at index, or NULL if index out of range.
    template<class Container>
    auto* indexOrNull(Container& container, Index index) { return index.isValid(container.size()) ? &container[index.index()] : nullptr; }

    // Get the value in the container at index, or a default value if index out of range.
    template<class Container, typename Value>
    auto indexOrDefault(Container& container, Index index, Value default_) { return index.isValid(container.size()) ? container[index.index()] : default_; }

} // namespace mip
