#include "scope_helper.hpp"

namespace mip
{
    namespace extras
    {
        ScopeHelper::ScopeHelper(std::function<void()> scopeFunction) :
            m_outOfScopeFunction(scopeFunction),
            m_canceled(false)
        {
        }

        ScopeHelper::~ScopeHelper()
        {
            if (!m_canceled)
            {
                m_outOfScopeFunction();
            }
        }

        void ScopeHelper::cancel()
        {
            m_canceled = true;
        }
    } //namespace extras
} //namespace mip