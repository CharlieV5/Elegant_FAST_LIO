/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/


#include "ikd_Tree_impl.h"

#include <pcl/point_types.h>


template class ikdtreeNS::KD_TREE<ikdtreeNS::ikdTree_PointType>;
template class ikdtreeNS::KD_TREE<pcl::PointXYZ>;
template class ikdtreeNS::KD_TREE<pcl::PointXYZI>;
template class ikdtreeNS::KD_TREE<pcl::PointXYZINormal>;