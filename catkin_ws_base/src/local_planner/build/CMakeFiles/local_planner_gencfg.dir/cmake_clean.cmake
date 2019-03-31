FILE(REMOVE_RECURSE
  "CMakeFiles/local_planner_gencfg"
  "devel/include/local_planner/local_planConfig.h"
  "devel/share/local_planner/docs/local_planConfig.dox"
  "devel/share/local_planner/docs/local_planConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/local_planner/cfg/local_planConfig.py"
  "devel/share/local_planner/docs/local_planConfig.wikidoc"
  "devel/include/local_planner/lfd_learnConfig.h"
  "devel/share/local_planner/docs/lfd_learnConfig.dox"
  "devel/share/local_planner/docs/lfd_learnConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/local_planner/cfg/lfd_learnConfig.py"
  "devel/share/local_planner/docs/lfd_learnConfig.wikidoc"
  "devel/include/local_planner/KRLSQConfig.h"
  "devel/share/local_planner/docs/KRLSQConfig.dox"
  "devel/share/local_planner/docs/KRLSQConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/local_planner/cfg/KRLSQConfig.py"
  "devel/share/local_planner/docs/KRLSQConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/local_planner_gencfg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
