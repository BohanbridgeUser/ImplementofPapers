#ifndef _ALGORITHM_FACTORY_H_
#define _ALGORITHM_FACTORY_H_

#include "algorithm.h"
#include "smoothness_constraint_segmentation.h"

class AlgorithmFactory
{
    public:
        using Creator = std::function<std::shared_ptr<Algorithm>()>;
        AlgorithmFactory();
        ~AlgorithmFactory()=default;

        static void Register_Algorithm(const std::string& name, Creator creator)
        {
            get_registry()[name] = creator;
        }

        static std::shared_ptr<Algorithm> Create_Algorithm(const std::string& name)
        {
            auto it = get_registry().find(name);
            if(it != get_registry().end())
                return it->second();
            else
            {
                std::cerr << "Algorithm not found : " + name << std::endl;
                return std::shared_ptr<Algorithm>();
            }
                
        }

    private:
        static std::unordered_map<std::string, Creator>& get_registry()
        {
            static std::unordered_map<std::string, Creator> registry;
            return registry;
        }

        static bool m_global_registed;
};

#endif