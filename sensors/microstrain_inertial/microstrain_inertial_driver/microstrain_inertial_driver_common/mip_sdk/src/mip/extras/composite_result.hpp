#pragma once

#include "descriptor_id.hpp"

#include "../mip_device.hpp"
#include "../mip_result.h"
#include "../definitions/descriptors.h"


#include <stddef.h>
#include <algorithm>

namespace mip
{

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Represents a list of zero or more actions and their results.
    ///
    class CompositeResult
    {
    public:
        struct Entry
        {
            CmdResult    result;         ///< Result of action.
            DescriptorId descriptor;     ///< Command or action that was executed.

            operator bool() const { return result; }
            bool operator!() const { return !result; }

            Entry(bool r, DescriptorId d={}) : result(r ? CmdResult::ACK_OK : CmdResult::STATUS_ERROR), descriptor(d) {}
            Entry(CmdResult r, DescriptorId d={}) : result(r), descriptor(d) {}
            Entry(C::mip_cmd_result r, DescriptorId d={}) : result(r), descriptor(d) {}
            //Entry(CmdResult r, CompositeDescriptor c) : result(r), descriptor(c) {}
        };

    public:
        CompositeResult() {}
        CompositeResult(bool success) : m_results{Entry{success, 0x0000}} {}
        CompositeResult(CmdResult result) : m_results{result} {}
        CompositeResult(CompositeDescriptor cmd, CmdResult result) : m_results{{result, cmd}} {}
        CompositeResult(const Entry& result) : m_results{result} {}

        bool isEmpty() const { return m_results.empty(); }
        bool notEmpty() const { return !m_results.empty(); }
        size_t count() const { return m_results.size(); }

        bool allSuccessful()  const { return std::all_of (m_results.begin(), m_results.end(), [](const Entry& r){ return  r.result.isAck(); }); }
        bool allFailed()      const { return std::all_of (m_results.begin(), m_results.end(), [](const Entry& r){ return !r.result.isAck(); }); }
        bool anySuccessful()  const { return std::any_of (m_results.begin(), m_results.end(), [](const Entry& r){ return  r.result.isAck(); }); }
        bool anyFailed()      const { return std::any_of (m_results.begin(), m_results.end(), [](const Entry& r){ return !r.result.isAck(); }); }
        bool noneSuccessful() const { return std::none_of(m_results.begin(), m_results.end(), [](const Entry& r){ return  r.result.isAck(); }); }
        bool noneFailed()     const { return std::none_of(m_results.begin(), m_results.end(), [](const Entry& r){ return !r.result.isAck(); }); }

        bool allMatch (CmdResult result) const { return std::all_of (m_results.begin(), m_results.end(), [result](const Entry& r){ return r.result == result; }); }
        bool anyMatch (CmdResult result) const { return std::any_of (m_results.begin(), m_results.end(), [result](const Entry& r){ return r.result == result; }); }
        bool noneMatch(CmdResult result) const { return std::none_of(m_results.begin(), m_results.end(), [result](const Entry& r){ return r.result == result; }); }

        operator bool() const { return noneFailed(); }
        bool operator!() const { return anyFailed(); }

        CmdResult summary() const
        {
            if (isEmpty()) return CmdResult::STATUS_NONE;
            if (count() == 1) return m_results.front().result;
            if (allSuccessful()) return CmdResult::ACK_OK;
            if (anyMatch(CmdResult::STATUS_TIMEDOUT)) return CmdResult::STATUS_TIMEDOUT;
            else return CmdResult::STATUS_ERROR;
        }

        void clear() { m_results.clear(); }

        // Append result to the list.
        //void append(bool success, uint8_t id=0) { m_results.push_back({success ? CmdResult::ACK_OK : CmdResult::STATUS_ERROR, 0x0000}); }
        void append(CmdResult result, CompositeDescriptor desc=0x0000) { m_results.push_back({result, desc}); }
        void append(Entry result) { m_results.push_back(result); }
        template<class MipType>
        void append(CmdResult result, uint16_t index=0) { append({result, DescriptorId(MipType::DESCRIPTOR, index)}); }

        // Append multiple results.
        void extend(const CompositeResult& other) { m_results.insert(m_results.end(), other.m_results.begin(), other.m_results.end()); }

        // Same as append but returns *this.
        CompositeResult& operator+=(bool result) { append(result); return *this; }
        CompositeResult& operator+=(CmdResult result) { append(result); return *this; }
        CompositeResult& operator+=(Entry result) { append(result); return *this; }

        // Same as append but returns the result.
        // Useful for code like `if( !results.appendAndCheckThis( doCommand() ) { return results; /* This specific command failed */ })`
        bool appendAndCheckThisCmd(bool result, uint32_t id) { append({result, id}); return result; }
        bool appendAndCheckThisCmd(CmdResult result, CompositeDescriptor desc) { append({result, desc}); return result; }
        bool appendAndCheckThisCmd(Entry result) { append(result); return result; }

        // Filter results (these would be a lot easier to implement with C++20's ranges filtering stuff)
        // Returns a new CompositeResult (or C++20 filter view) with only the filtered results.
        // Todo: Implement these as necessary.
        //auto filterByStatus(CmdResult result) const {}
        //auto filterByStatusNot(CmdResult result) const {}
        //auto filterSuccessful() {}
        //auto filterFailed() {}
        //auto filterById(CompositeDescriptor) {}

        const Entry& first() const { return m_results.front(); }
        Entry& first() { return m_results.front(); }

        const Entry& last() const { return m_results.back(); }
        Entry& last() { return m_results.back(); }

        CmdResult firstResult() const { return m_results.front().result; }
        CmdResult lastResult() const { return m_results.back().result; }

        auto begin() { return m_results.begin(); }
        auto end()   { return m_results.end();   }

        auto begin() const { return m_results.begin(); }
        auto end()   const { return m_results.end();   }

    private:
        std::vector<Entry> m_results;
    };


    template<class Cmd, class... Args>
    CompositeResult::Entry runCommandEx(DeviceInterface& device, const Cmd& cmd, Args&&... args)
    {
        CmdResult result = device.runCommand(cmd, std::forward<Args>(args)...);

        return {result, {Cmd::DESCRIPTOR_SET, Cmd::FIELD_DESCRIPTOR}};
    }

} // namespace mip
