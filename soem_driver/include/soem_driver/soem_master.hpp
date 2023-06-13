#ifndef SOEM_DRIVER__SOEM_MASTER_HPP_
#define SOEM_DRIVER__SOEM_MASTER_HPP_

#include <memory>
#include <vector>
#include <map>


namespace soem_master
{
    class SOEMMaster
    {
        public:
            SOEMMaster();
            ~SOEMMaster();

            void setup(const std::string& interface);
            
    };
} // namespace soem_master

#endif // SOEM_DRIVER__SOEM_MASTER_HPP_