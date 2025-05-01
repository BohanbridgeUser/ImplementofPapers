#include "algorithm_factory.h"
#include "smoothness_constraint_segmentation.h"
#include "PoissonReconstruction.h"

bool AlgorithmFactory::m_global_registed = false;

AlgorithmFactory::AlgorithmFactory()
{
    if(!m_global_registed)
    {
        m_global_registed = true;
        Register_Algorithm("smooth", [](){
            return std::make_shared<SmoothnessConstraintSegmentation>();
        });
        Register_Algorithm("Poisson", [](){
            return std::make_shared<PoissonReconstruction>();
        });
    }
}