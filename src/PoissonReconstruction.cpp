#include "PoissonReconstruction.h"

/// @brief public:
/// @name Type Define
/// @{
/// @}
/// @name Life Circle
/// @{

PoissonReconstruction::PoissonReconstruction()
{
    if(spdlog::get("PoissonReconstruction")){
        p_logger = spdlog::get("PoissonReconstruction");
    }
    else{
        p_logger = spdlog::stdout_color_mt("PoissonReconstruction");
    }

    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] [%n] [%l] %v");

    m_PWNs = Vector_PWN();
    m_filename = std::string();
    m_points_set = Point_set();
    p_octree = nullptr;
    m_normal_reversed = false;
    m_alphaOS = std::unordered_map<NodeIndex, Eigen::Vector3d>();
    m_num_of_nodes = 0;

    m_polygon_points = std::vector<Point>();
    m_polygon_soup = std::vector<std::vector<int>>();


    m_triTable = {{
        {{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1}},
        {{3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1}},
        {{3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1}},
        {{3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1}},
        {{9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1}},
        {{1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1}},
        {{9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1}},
        {{2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1}},
        {{8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1}},
        {{9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1}},
        {{4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1}},
        {{3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1}},
        {{1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1}},
        {{4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1}},
        {{4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1}},
        {{9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1}},
        {{1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1}},
        {{5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1}},
        {{2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1}},
        {{9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1}},
        {{0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1}},
        {{2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1}},
        {{10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1}},
        {{4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1}},
        {{5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1}},
        {{5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1}},
        {{9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1}},
        {{0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1}},
        {{1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
        {{9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1}},
        {{10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1}},
        {{8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1}},
        {{2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1}},
        {{7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1}},
        {{9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1}},
        {{2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1}},
        {{11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1}},
        {{9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1}},
        {{5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1}},
        {{11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1}},
        {{11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1    }},
        {{10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1   }},
        {{0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1         }},
        {{1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1       }},
        {{1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1          }},
        {{9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1          }},
        {{5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1             }},
        {{2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1     }},
        {{11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1       }},
        {{0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1        }},
        {{5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1          }},
        {{6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1         }},
        {{0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1          }},
        {{3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1            }},
        {{6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1        }},
        {{5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1         }},
        {{1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1         }},
        {{10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1            }},
        {{6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1          }},
        {{1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1             }},
        {{8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1             }},
        {{7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1                }},
        {{3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1        }},
        {{5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1           }},
        {{0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1           }},
        {{9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1            }},
        {{8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1           }},
        {{5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1           }},
        {{0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1               }},
        {{6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1           }},
        {{10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1     }},
        {{4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1        }},
        {{10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1        }},
        {{8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1            }},
        {{1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1          }},
        {{3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1             }},
        {{0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1       }},
        {{8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1          }},
        {{10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1       }},
        {{0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1          }},
        {{3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1           }},
        {{6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1             }},
        {{9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1            }},
        {{8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1              }},
        {{3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1         }},
        {{6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1       }},
        {{0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1          }},
        {{10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1           }},
        {{10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1        }},
        {{1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1             }},
        {{2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1                }},
        {{7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1          }},
        {{7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1       }},
        {{2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1          }},
        {{2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1             }},
        {{1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1             }},
        {{11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1          }},
        {{8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1               }},
        {{0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1           }},
        {{7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1   }},
        {{7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1   }},
        {{3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1         }},
        {{10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1     }},
        {{1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1        }},
        {{2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1        }},
        {{6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1         }},
        {{7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1       }},
        {{7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1          }},
        {{2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1          }},
        {{1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1             }},
        {{10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1        }},
        {{10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1           }},
        {{0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1          }},
        {{7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1       }},
        {{6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1         }},
        {{8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1         }},
        {{9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1            }},
        {{6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1        }},
        {{1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1          }},
        {{4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1          }},
        {{10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1             }},
        {{8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1          }},
        {{0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1       }},
        {{1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1             }},
        {{1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1          }},
        {{8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1            }},
        {{10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1        }},
        {{4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1              }},
        {{10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1     }},
        {{4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1         }},
        {{5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1         }},
        {{11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1            }},
        {{9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1        }},
        {{6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1           }},
        {{7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1          }},
        {{3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1              }},
        {{7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1          }},
        {{9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1             }},
        {{3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1             }},
        {{6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1                }},
        {{9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1            }},
        {{1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1               }},
        {{4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1           }},
        {{7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1         }},
        {{6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1        }},
        {{3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1            }},
        {{0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1          }},
        {{6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1         }},
        {{1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1         }},
        {{0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1             }},
        {{11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1             }},
        {{6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1          }},
        {{5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1             }},
        {{9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1          }},
        {{1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1                }},
        {{1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1       }},
        {{1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1               }},
        {{10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1           }},
        {{0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1   }},
        {{11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1    }},
        {{11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1       }},
        {{5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1       }},
        {{10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1          }},
        {{11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1        }},
        {{0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1            }},
        {{9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1            }},
        {{7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1               }},
        {{2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1         }},
        {{8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1            }},
        {{9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1           }},
        {{9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1               }},
        {{1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1       }},
        {{0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1          }},
        {{9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1          }},
        {{9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1       }},
        {{5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1       }},
        {{5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1         }},
        {{0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1         }},
        {{10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1            }},
        {{2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1            }},
        {{0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1           }},
        {{0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1              }},
        {{9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1            }},
        {{5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1         }},
        {{3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1              }},
        {{5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1            }},
        {{8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1          }},
        {{0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1       }},
        {{8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1             }},
        {{9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1    }},
        {{4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1      }},
        {{0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1          }},
        {{1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1         }},
        {{3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1            }},
        {{4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1          }},
        {{9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1             }},
        {{11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1        }},
        {{11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1           }},
        {{2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1            }},
        {{9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1              }},
        {{3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1           }},
        {{1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1          }},
        {{4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1             }},
        {{4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1       }},
        {{4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1    }},
        {{9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1    }},
        {{3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1       }},
        {{0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1      }},
        {{3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1    }},
        {{1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1       }},
        {{3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1           }},
        {{0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1     }},
        {{3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1   }},
        {{2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1        }},
        {{9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1      }},
        {{2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1           }},
        {{1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1   }},
        {{1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1       }},
        {{0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1    }},
        {{0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1    }},
        {{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }}
    }};
    
    m_edgeTable = {{
        {{0,1}}, {{1,2}}, {{2,3}}, {{3,0}},
        {{4,5}}, {{5,6}}, {{6,7}}, {{7,4}},
        {{0,4}}, {{1,5}}, {{2,6}}, {{3,7}}
    }};
}

/// @}
/// @name Operators
/// @{
/// @}
/// @name Operations
/// @{

bool PoissonReconstruction::reverseNormal()
{
    m_normal_reversed = !m_normal_reversed;
    for(int i=0;i<m_PWNs.size();++i)
        m_PWNs[i].second = -1.0 * m_PWNs[i].second; 
    return true;
}

bool PoissonReconstruction::buildOctree()
{
    std::vector<Point> points_vec;
    for(int i=0;i<m_PWNs.size();++i)
        points_vec.push_back(m_PWNs[i].first);
    Bbox_3 bbox3 = CGAL::bbox_3(points_vec.begin(), points_vec.end());
    m_virtual_points.insert({m_co_bbox * bbox3.xmax(), m_co_bbox * bbox3.ymax(), m_co_bbox * bbox3.zmax()});
    m_virtual_points.insert({m_co_bbox * bbox3.xmin(), m_co_bbox * bbox3.ymax(), m_co_bbox * bbox3.zmax()});
    m_virtual_points.insert({m_co_bbox * bbox3.xmax(), m_co_bbox * bbox3.ymin(), m_co_bbox * bbox3.zmax()});
    m_virtual_points.insert({m_co_bbox * bbox3.xmax(), m_co_bbox * bbox3.ymax(), m_co_bbox * bbox3.zmin()});
    m_virtual_points.insert({m_co_bbox * bbox3.xmin(), m_co_bbox * bbox3.ymin(), m_co_bbox * bbox3.zmax()});
    m_virtual_points.insert({m_co_bbox * bbox3.xmax(), m_co_bbox * bbox3.ymin(), m_co_bbox * bbox3.zmin()});
    m_virtual_points.insert({m_co_bbox * bbox3.xmin(), m_co_bbox * bbox3.ymax(), m_co_bbox * bbox3.zmin()});
    m_virtual_points.insert({m_co_bbox * bbox3.xmin(), m_co_bbox * bbox3.ymin(), m_co_bbox * bbox3.zmin()});

    m_points_set.clear();
    for(int i=0;i<m_PWNs.size();++i)
        m_points_set.insert(m_PWNs[i].first, m_PWNs[i].second);
    for(auto vpoint:m_virtual_points)
        m_points_set.insert(vpoint);
    p_octree = std::make_shared<Octree>(m_points_set,m_points_set.point_map());
    p_octree->refine(CGAL::Orthtrees::Maximum_contained_elements(1));
    /* refine the leaves that contain sample point to max depth D */
    int maxDepth = p_octree->depth();
    for(auto node : p_octree->traverse<LeavesTraversal>())
    {
        auto data = p_octree->data(node);
        if(data && data.size() > 0)
            if(m_virtual_points.find(m_points_set.point(data[0])) != m_virtual_points.end())
                continue;
        while(data && data.size() > 0 && p_octree->depth(node) < maxDepth)
        {
            int befor = p_octree->depth(node);
            p_octree->split(node);
            /* every leaf has one point only */
            for(int i=0;i<8;++i)
            {
                NodeIndex cnode = p_octree->child(node,i);
                auto cdata = p_octree->data(cnode);
                if(cdata && cdata.size() > 0)
                {
                    node = cnode;
                    data = cdata;
                    // p_logger->info("Node : {}  before : {}  after : {}", node, befor, p_octree->depth(node));
                    break;
                }
            }
        }
    }
    p_octree->grade();
    int newmaxDepth = p_octree->depth();
    p_logger->info("Old maxDepth {}, new maxDepth {}", maxDepth, newmaxDepth);
    // for(auto node : p_octree->traverse<LeavesTraversal>())
    // {
    //     auto data = p_octree->data(node);
    //     p_logger->info("data size {} , depth {}", data.size(), p_octree->depth(node));
    // }
    
    for(auto node : p_octree->traverse<PreorderTraversal>()) {
        m_num_of_nodes++;
        m_overlap_map[node] = std::vector<NodeIndex>();
        m_overlap_set[node] = std::unordered_set<NodeIndex>();
        m_overlap_map_map[node] = std::unordered_map<NodeIndex, Bbox_3>();
    }

    for(int i=0;i<p_octree->depth();++i)
        outputBboxD(i);

    return true;
}

bool PoissonReconstruction::buildVectorField()
{
    int cnt = 0;
    auto index_range = p_octree->traverse<LeavesTraversal>();
    for(auto it = index_range.begin();it != index_range.end(); ++it)
    {
        NodeIndex node = *it;
        if(p_octree->data(node).size() > 0)
        {
            int pid = p_octree->data(node)[0];
            Point& point = m_points_set.point(p_octree->data(node)[0]);
            if(m_virtual_points.find(point) != m_virtual_points.end())
                continue;
            Vector& sn = m_points_set.normal(p_octree->data(node)[0]);
            Point bboxCen = calBboxCen(node);
            // p_logger->info("Point : {} {} {} ", point.x(), point.y(), point.z());
            // p_logger->info("Center of bbox : {} {} {} ", bboxCen.x(), bboxCen.y(), bboxCen.z());
            // p_logger->info("Depth : {} ", p_octree->depth(node));
                
            /**
             *            3 *
             *              |  * 4
             *              | /                  y+
             *              |/                   *
             *     0 *------+------* 1           |
             *             /|                    |
             *            / |                    +-----* x+
             *         5 *  |                   /
             *              * 2                /
             *                                * z+
             *
             * This lookup table may also be helpful:
             *
             * | Direction | bitset | number | Enum  |
             * | --------- | ------ | ------ | ----- |
             * | `-x`      | 000    | 0      | LEFT  |
             * | `+x`      | 001    | 1      | RIGHT |
             * | `-y`      | 010    | 2      | DOWN  |
             * | `+y`      | 011    | 3      | UP    |
             * | `-z`      | 100    | 4      | BACK  |
             * | `+z`      | 101    | 5      | FRONT |
             *  
             * @details The store sequence of node center. y axis is upward. Local coordinate origin is 7
             * | 2<-1<-0 |
             * | bitset  |  Loc   |       | Num |
             * |  000    | L D B  | T F R |  0  |
             * |  001    | R D B  | T F L |  1  |
             * |  010    | L T B  | D F R |  2  |
             * |  100    | L D F  | T B R |  4  |
             * |  011    | R T B  | D F L |  3  |
             * |  101    | R D F  | T B L |  5  |
             * |  110    | L T F  | D B R |  6  |
             * |  111    | R T F  | D B L |  7  |
             */
            std::vector<std::pair<NodeIndex,Point>> neighborCenter;
            int depth = p_octree->depth(node);
            std::bitset<3> loc(0);
            if(point.x() > bboxCen.x())
                loc.set(0,1);
            if(point.y() > bboxCen.y())
                loc.set(1,1);
            if(point.z() > bboxCen.z())
                loc.set(2,1);
            switch (loc.to_ulong())
            {
                case 0:{
                        // std::cout << "case 0\n";
                        findAdjacentNodeCenter(node, 0, 0, 0, depth);
                        findAdjacentNodeCenter(node, 4, 4, 4, depth);
                        findAdjacentNodeCenter(node, 4, 0, 0, depth);
                        findAdjacentNodeCenter(node, 2, 2, 2, depth);
                        findAdjacentNodeCenter(node, 2, 0, 0, depth);
                        findAdjacentNodeCenter(node, 2, 4, 4, depth);
                        findAdjacentNodeCenter(node, 2, 4, 0, depth);
                        break;
                    }
                case 1:{
                        // std::cout << "case 1\n \n";
                        findAdjacentNodeCenter(node, 1, 1, 1, depth);
                        findAdjacentNodeCenter(node, 4, 1, 1, depth);
                        findAdjacentNodeCenter(node, 4, 4, 4, depth);
                        findAdjacentNodeCenter(node, 2, 1, 1, depth);
                        findAdjacentNodeCenter(node, 2, 2, 2, depth);
                        findAdjacentNodeCenter(node, 2, 4, 1, depth);
                        findAdjacentNodeCenter(node, 2, 4, 4, depth);
                        break;
                    }
                case 2:{
                        // std::cout << "case 2\n \n";
                        findAdjacentNodeCenter(node, 3, 3, 3, depth);
                        findAdjacentNodeCenter(node, 3, 0, 0, depth);
                        findAdjacentNodeCenter(node, 3, 4, 4, depth);
                        findAdjacentNodeCenter(node, 3, 4, 0, depth);
                        findAdjacentNodeCenter(node, 0, 0, 0, depth);
                        findAdjacentNodeCenter(node, 4, 4, 4, depth);
                        findAdjacentNodeCenter(node, 4, 0, 0, depth);
                        break;
                    }
                case 3:{
                        // std::cout << "case 3\n \n";
                        findAdjacentNodeCenter(node, 3, 1, 1, depth);
                        findAdjacentNodeCenter(node, 3, 3, 3, depth);
                        findAdjacentNodeCenter(node, 3, 4, 1, depth);
                        findAdjacentNodeCenter(node, 3, 4, 4, depth);
                        findAdjacentNodeCenter(node, 1, 1, 1, depth);
                        findAdjacentNodeCenter(node, 4, 1, 1, depth);  
                        findAdjacentNodeCenter(node, 4, 4, 4, depth);
                        break;
                    }
                case 4:{
                        // std::cout << "case 4\n \n";
                        findAdjacentNodeCenter(node, 5, 5, 5, depth);
                        findAdjacentNodeCenter(node, 5, 0, 0, depth);
                        findAdjacentNodeCenter(node, 0, 0, 0, depth);
                        findAdjacentNodeCenter(node, 2, 5, 5, depth);
                        findAdjacentNodeCenter(node, 2, 5, 0, depth);
                        findAdjacentNodeCenter(node, 2, 2, 2, depth);
                        findAdjacentNodeCenter(node, 2, 0, 0, depth);
                        break;
                    }
                case 5:{
                        // std::cout << "case 5\n \n";
                        findAdjacentNodeCenter(node, 5, 1, 1, depth);
                        findAdjacentNodeCenter(node, 5, 5, 5, depth);
                        findAdjacentNodeCenter(node, 1, 1, 1, depth);
                        findAdjacentNodeCenter(node, 2, 5, 1, depth);
                        findAdjacentNodeCenter(node, 2, 5, 5, depth);
                        findAdjacentNodeCenter(node, 2, 1, 1, depth);
                        findAdjacentNodeCenter(node, 2, 2, 2, depth);
                        break;
                    }
                case 6:{
                        // std::cout << "case 6\n \n";
                        findAdjacentNodeCenter(node, 3, 5, 5, depth);
                        findAdjacentNodeCenter(node, 3, 5, 0, depth);
                        findAdjacentNodeCenter(node, 3, 3, 3, depth);
                        findAdjacentNodeCenter(node, 3, 0, 0, depth);
                        findAdjacentNodeCenter(node, 5, 5, 5, depth);
                        findAdjacentNodeCenter(node, 5, 0, 0, depth);
                        findAdjacentNodeCenter(node, 0, 0, 0, depth);
                        break;
                    }
                case 7:{
                        // std::cout << "case 7\n \n";
                        findAdjacentNodeCenter(node, 3, 5, 1, depth);
                        findAdjacentNodeCenter(node, 3, 5, 5, depth);
                        findAdjacentNodeCenter(node, 3, 1, 1, depth);
                        findAdjacentNodeCenter(node, 3, 3, 3, depth);
                        findAdjacentNodeCenter(node, 5, 1, 1, depth);
                        findAdjacentNodeCenter(node, 5, 5, 5, depth);
                        findAdjacentNodeCenter(node, 1, 1, 1, depth);
                        break;
                    }
                default:{
                    // std::cout << "Unknown case \n \n";
                    break;
                }    
            }
        }        
    }

    for(auto node : p_octree->traverse<LeavesTraversal>())
    {
        if(p_octree->data(node).size() > 0)
        {
            int pid = p_octree->data(node)[0];
            Point& point = m_points_set.point(p_octree->data(node)[0]);
            if(m_virtual_points.find(point) != m_virtual_points.end())
                continue;
            Vector& sn = m_points_set.normal(p_octree->data(node)[0]);
            Point bboxCen = calBboxCen(node);
            // p_logger->info("Point : {} {} {} ", point.x(), point.y(), point.z());
            // p_logger->info("Center of bbox : {} {} {} ", bboxCen.x(), bboxCen.y(), bboxCen.z());
            // p_logger->info("Depth : {} ", p_octree->depth(node));
            /**
             *            3 *
             *              |  * 4
             *              | /                  y+
             *              |/                   *
             *     0 *------+------* 1           |
             *             /|                    |
             *            / |                    +-----* x+
             *         5 *  |                   /
             *              * 2                /
             *                                * z+
             *
             * This lookup table may also be helpful:
             *
             * | Direction | bitset | number | Enum  |
             * | --------- | ------ | ------ | ----- |
             * | `-x`      | 000    | 0      | LEFT  |
             * | `+x`      | 001    | 1      | RIGHT |
             * | `-y`      | 010    | 2      | DOWN  |
             * | `+y`      | 011    | 3      | UP    |
             * | `-z`      | 100    | 4      | BACK  |
             * | `+z`      | 101    | 5      | FRONT |
             *  
             * @details The store sequence of node center. y axis is upward. Local coordinate origin is 7
             * | 2<-1<-0 |
             * | bitset  |  Loc   |       | Num |
             * |  000    | L D B  | T F R |  0  |
             * |  001    | R D B  | T F L |  1  |
             * |  010    | L T B  | D F R |  2  |
             * |  100    | L D F  | T B R |  4  |
             * |  011    | R T B  | D F L |  3  |
             * |  101    | R D F  | T B L |  5  |
             * |  110    | L T F  | D B R |  6  |
             * |  111    | R T F  | D B L |  7  |
             */
            std::vector<std::pair<NodeIndex,Point>> neighborCenter;
            int depth = p_octree->depth(node);
            std::bitset<3> loc(0);
            if(point.x() > bboxCen.x())
                loc.set(0,1);
            if(point.y() > bboxCen.y())
                loc.set(1,1);
            if(point.z() > bboxCen.z())
                loc.set(2,1);
            switch (loc.to_ulong())
            {
                case 0:{
                        // std::cout << "case 0\n";
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 0, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 2, 2, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 4, 0, depth));
                        break;
                    }
                case 1:{
                        // std::cout << "case 1\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 1, 1, 1, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 2, 2, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 4, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 4, 4, depth));
                        break;
                    }
                case 2:{
                        // std::cout << "case 2\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 3, 3, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 4, 0, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 0, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 0, 0, depth));
                        break;
                    }
                case 3:{
                        // std::cout << "case 3\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 3, 3, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 4, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 1, 1, 1, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 1, 1, depth));  
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 4, 4, depth));
                        break;
                    }
                case 4:{
                        // std::cout << "case 4\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 0, 0, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 0, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 5, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 2, 2, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 0, 0, depth));
                        break;
                    }
                case 5:{
                        // std::cout << "case 5\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 1, 1, 1, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 5, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 2, 2, depth));
                        break;
                    }
                case 6:{
                        // std::cout << "case 6\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 5, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 3, 3, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 0, 0, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 0, 0, 0, depth));
                        break;
                    }
                case 7:{
                        // std::cout << "case 7\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 5, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 3, 3, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 1, 1, 1, depth));
                        neighborCenter.push_back({node,bboxCen});
                        break;
                    }
                default:{
                    // std::cout << "Unknown case \n \n";
                    break;
                }    
            }
            if(cnt <= 200)
                outputTriInter(point, neighborCenter);
            trilinearInterpolation(neighborCenter, point, sn);
            cnt++;
        }        
    }
    p_octree->grade();
    // calOverlap();
    calOverlapRecursive();
    return true;
}

bool PoissonReconstruction::buildVectorFieldD()
{
    int cnt = 0;
    auto index_range = p_octree->traverse<LeavesTraversal>();
    for(auto it = index_range.begin();it != index_range.end(); ++it)
    {
        NodeIndex node = *it;
        if(p_octree->data(node).size() > 0)
        {
            int pid = p_octree->data(node)[0];
            Point& point = m_points_set.point(p_octree->data(node)[0]);
            if(m_virtual_points.find(point) != m_virtual_points.end())
                continue;
            Vector& sn = m_points_set.normal(p_octree->data(node)[0]);
            Point bboxCen = calBboxCen(node);
            std::vector<std::pair<NodeIndex,Point>> neighborCenter;
            int depth = p_octree->depth(node);
            std::bitset<3> loc(0);
            if(point.x() > bboxCen.x())
                loc.set(0,1);
            if(point.y() > bboxCen.y())
                loc.set(1,1);
            if(point.z() > bboxCen.z())
                loc.set(2,1);
            switch (loc.to_ulong())
            {
                case 0:{
                        // std::cout << "case 0\n";
                        findAdjacentNodeCenter(node, 0, 0, 0, depth);
                        findAdjacentNodeCenter(node, 4, 4, 4, depth);
                        findAdjacentNodeCenter(node, 4, 0, 0, depth);
                        findAdjacentNodeCenter(node, 2, 2, 2, depth);
                        findAdjacentNodeCenter(node, 2, 0, 0, depth);
                        findAdjacentNodeCenter(node, 2, 4, 4, depth);
                        findAdjacentNodeCenter(node, 2, 4, 0, depth);
                        break;
                    }
                case 1:{
                        // std::cout << "case 1\n \n";
                        findAdjacentNodeCenter(node, 1, 1, 1, depth);
                        findAdjacentNodeCenter(node, 4, 1, 1, depth);
                        findAdjacentNodeCenter(node, 4, 4, 4, depth);
                        findAdjacentNodeCenter(node, 2, 1, 1, depth);
                        findAdjacentNodeCenter(node, 2, 2, 2, depth);
                        findAdjacentNodeCenter(node, 2, 4, 1, depth);
                        findAdjacentNodeCenter(node, 2, 4, 4, depth);
                        break;
                    }
                case 2:{
                        // std::cout << "case 2\n \n";
                        findAdjacentNodeCenter(node, 3, 3, 3, depth);
                        findAdjacentNodeCenter(node, 3, 0, 0, depth);
                        findAdjacentNodeCenter(node, 3, 4, 4, depth);
                        findAdjacentNodeCenter(node, 3, 4, 0, depth);
                        findAdjacentNodeCenter(node, 0, 0, 0, depth);
                        findAdjacentNodeCenter(node, 4, 4, 4, depth);
                        findAdjacentNodeCenter(node, 4, 0, 0, depth);
                        break;
                    }
                case 3:{
                        // std::cout << "case 3\n \n";
                        findAdjacentNodeCenter(node, 3, 1, 1, depth);
                        findAdjacentNodeCenter(node, 3, 3, 3, depth);
                        findAdjacentNodeCenter(node, 3, 4, 1, depth);
                        findAdjacentNodeCenter(node, 3, 4, 4, depth);
                        findAdjacentNodeCenter(node, 1, 1, 1, depth);
                        findAdjacentNodeCenter(node, 4, 1, 1, depth);  
                        findAdjacentNodeCenter(node, 4, 4, 4, depth);
                        break;
                    }
                case 4:{
                        // std::cout << "case 4\n \n";
                        findAdjacentNodeCenter(node, 5, 5, 5, depth);
                        findAdjacentNodeCenter(node, 5, 0, 0, depth);
                        findAdjacentNodeCenter(node, 0, 0, 0, depth);
                        findAdjacentNodeCenter(node, 2, 5, 5, depth);
                        findAdjacentNodeCenter(node, 2, 5, 0, depth);
                        findAdjacentNodeCenter(node, 2, 2, 2, depth);
                        findAdjacentNodeCenter(node, 2, 0, 0, depth);
                        break;
                    }
                case 5:{
                        // std::cout << "case 5\n \n";
                        findAdjacentNodeCenter(node, 5, 1, 1, depth);
                        findAdjacentNodeCenter(node, 5, 5, 5, depth);
                        findAdjacentNodeCenter(node, 1, 1, 1, depth);
                        findAdjacentNodeCenter(node, 2, 5, 1, depth);
                        findAdjacentNodeCenter(node, 2, 5, 5, depth);
                        findAdjacentNodeCenter(node, 2, 1, 1, depth);
                        findAdjacentNodeCenter(node, 2, 2, 2, depth);
                        break;
                    }
                case 6:{
                        // std::cout << "case 6\n \n";
                        findAdjacentNodeCenter(node, 3, 5, 5, depth);
                        findAdjacentNodeCenter(node, 3, 5, 0, depth);
                        findAdjacentNodeCenter(node, 3, 3, 3, depth);
                        findAdjacentNodeCenter(node, 3, 0, 0, depth);
                        findAdjacentNodeCenter(node, 5, 5, 5, depth);
                        findAdjacentNodeCenter(node, 5, 0, 0, depth);
                        findAdjacentNodeCenter(node, 0, 0, 0, depth);
                        break;
                    }
                case 7:{
                        // std::cout << "case 7\n \n";
                        findAdjacentNodeCenter(node, 3, 5, 1, depth);
                        findAdjacentNodeCenter(node, 3, 5, 5, depth);
                        findAdjacentNodeCenter(node, 3, 1, 1, depth);
                        findAdjacentNodeCenter(node, 3, 3, 3, depth);
                        findAdjacentNodeCenter(node, 5, 1, 1, depth);
                        findAdjacentNodeCenter(node, 5, 5, 5, depth);
                        findAdjacentNodeCenter(node, 1, 1, 1, depth);
                        break;
                    }
                default:{
                    // std::cout << "Unknown case \n \n";
                    break;
                }    
            }
        }        
    }

    for(auto node : p_octree->traverse<LeavesTraversal>())
    {
        if(p_octree->data(node).size() > 0)
        {
            int pid = p_octree->data(node)[0];
            Point& point = m_points_set.point(p_octree->data(node)[0]);
            if(m_virtual_points.find(point) != m_virtual_points.end())
                continue;
            Vector& sn = m_points_set.normal(p_octree->data(node)[0]);
            Point bboxCen = calBboxCen(node);
            std::vector<std::pair<NodeIndex,Point>> neighborCenter;
            int depth = p_octree->depth(node);
            std::bitset<3> loc(0);
            if(point.x() > bboxCen.x())
                loc.set(0,1);
            if(point.y() > bboxCen.y())
                loc.set(1,1);
            if(point.z() > bboxCen.z())
                loc.set(2,1);
            switch (loc.to_ulong())
            {
                case 0:{
                        // std::cout << "case 0\n";
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 0, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 2, 2, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 4, 0, depth));
                        break;
                    }
                case 1:{
                        // std::cout << "case 1\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 1, 1, 1, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 2, 2, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 4, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 4, 4, depth));
                        break;
                    }
                case 2:{
                        // std::cout << "case 2\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 3, 3, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 4, 0, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 0, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 0, 0, depth));
                        break;
                    }
                case 3:{
                        // std::cout << "case 3\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 3, 3, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 4, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 4, 4, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 1, 1, 1, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 1, 1, depth));  
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 4, 4, 4, depth));
                        break;
                    }
                case 4:{
                        // std::cout << "case 4\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 0, 0, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 0, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 5, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 2, 2, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 0, 0, depth));
                        break;
                    }
                case 5:{
                        // std::cout << "case 5\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 1, 1, 1, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 5, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 2, 2, 2, depth));
                        break;
                    }
                case 6:{
                        // std::cout << "case 6\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 5, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 3, 3, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 0, 0, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 0, 0, depth));
                        neighborCenter.push_back({node,bboxCen});
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 0, 0, 0, depth));
                        break;
                    }
                case 7:{
                        // std::cout << "case 7\n \n";
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 5, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 3, 3, 3, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 1, 1, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 5, 5, 5, depth));
                        neighborCenter.push_back(findAdjacentNodeCenter(node, 1, 1, 1, depth));
                        neighborCenter.push_back({node,bboxCen});
                        break;
                    }
                default:{
                    // std::cout << "Unknown case \n \n";
                    break;
                }    
            }
            trilinearInterpolation(neighborCenter, point, sn);
            cnt++;
        }        
    }
    calOverlapD();
    return true;
}

bool PoissonReconstruction::computeV()
{
    m_V = Eigen::VectorXd::Zero(m_num_of_nodes);
    p_logger->info("Num nodes : {}", m_num_of_nodes);
    for(int i=0;i<m_overlap_vec.size();++i)
    {
        NodeIndex node1ID = i;
        double v0i = 0.0;
        for(auto node2 : m_overlap_vec[i])
        {
            if(m_alphaOS.find(node2.first) != m_alphaOS.end())
            {
                NodeIndex node2ID = node2.first;
                Bbox_3 overlap = node2.second;
                v0i += calv0(node2ID, node1ID, overlap);
            }
        }
        m_V(node1ID) = v0i;
    }
    p_logger->info("m_V calculation done");
    return true;
}

bool PoissonReconstruction::computeVD()
{
    m_V = Eigen::VectorXd::Zero(m_overlap.size());
    p_logger->info("Num nodes : {}", m_overlap.size());
    m_F02Node.clear();
    m_Node2F0.resize(m_overlap.size());
    int cnt = 0;
    for(auto node1 : m_overlap)
    {
        NodeIndex node1ID = node1.first;
        m_F02Node[node1ID] = cnt;
        m_Node2F0[cnt] = node1ID;
        double v0i = 0.0;
        for(auto node2 : node1.second)
        {
            NodeIndex node2ID = node2.first;
            if(m_alphaOS.find(node2ID) != m_alphaOS.end())
            {
                Bbox_3 overlap = node2.second;
                v0i += calv0(node2ID, node1ID, overlap);
            }
        }
        m_V(cnt++) = v0i;
    }
    p_logger->info("m_V calculation done");
    return true;
}

bool PoissonReconstruction::computeV2()
{
    m_V = Eigen::VectorXd::Zero(m_overlap_map_map.size());
    p_logger->info("Num nodes : {}", m_overlap_map_map.size());
    m_F02Node.clear();
    m_Node2F0.resize(m_overlap_map_map.size());
    int cnt = 0;
    for(auto itemAlphaOS : m_alphaOS) {
        NodeIndex node1 = itemAlphaOS.first;
        // p_logger->info("Node1 : {}", node1);
        for(auto itemNode : m_overlap_map_map[node1]) {
            NodeIndex node2 = itemNode.first;
            Bbox_3 overlap = itemNode.second;
            m_V(node2) += calv0(node1, node2, overlap);
            // p_logger->info("vo : {} Node1 : {}, Node2 : {}  ", calv0(node1, node2, overlap), node1, node2);
        }
    }

    // for(auto node1 : p_octree->traverse<LeavesTraversal>()) {
    //     auto node1Data = p_octree->data(node1);
    //     if(node1Data && node1Data.size() == 0 )
    //         continue;
    //     if(m_virtual_points.find(m_points_set.point(node1Data[0])) != m_virtual_points.end())
    //         continue;
    //     Bbox_3 bbox1 = p_octree->bbox(node1).bbox();
    //     bbox1.scale(3.0);
    //     for(auto node2 : m_overlap_set[node1]) {
    //         if(m_alphaOS.find(node2) != m_alphaOS.end()) {
    //             Bbox_3 bbox2 = p_octree->bbox(node2).bbox();
    //             const auto result = CGAL::intersection(bbox1,bbox2);
    //             if(result) {
    //                 if(const Bbox_3* interbbox = std::get_if<Bbox_3>(&*result)) {                      
    //                 }
    //             }
    //         }
    //     }
    // }
    p_logger->info("m_V calculation done");
    return true;
}

bool PoissonReconstruction::computeL()
{
    m_L = Eigen::MatrixXd::Zero(m_num_of_nodes, m_num_of_nodes);
    sm_L = Eigen::SparseMatrix<double>(m_num_of_nodes,m_num_of_nodes);
    for(int i=0;i<m_overlap_vec.size();++i)
    {
        NodeIndex node1ID = i;
        double l0i = 0.0;
        for(auto node2 : m_overlap_vec[i])
        {
            NodeIndex node2ID = node2.first;
            Bbox_3 overlap = node2.second;
            l0i += call0(node2ID, node1ID, overlap);
            sm_L.insert(node1ID,node2ID) = l0i;
        }
    }
    p_logger->info("m_L calculation done");
    return true;
}

bool PoissonReconstruction::computeLD()
{
    sm_L = Eigen::SparseMatrix<double>(m_overlap.size(), m_overlap.size());
    Bbox_3 overlap5860;
    Bbox_3 overlap6058;
    for(auto node1 : m_overlap)
    {
        NodeIndex node1ID = node1.first;
        for(auto node2 : node1.second)
        {
            NodeIndex node2ID = node2.first;
            Bbox_3 overlap = node2.second;
            double l0i = call0(node2ID, node1ID, overlap);
            // Bbox_3 overlap5860;
            // Bbox_3 overlap6058;
            // if(node2ID == 14958 && node1ID == 14960)
            // {
            //     p_logger->info("Node 14958 Node 14960 : {}", l0i);
            //     overlap5860 = overlap;
            // }
            // if(node2ID == 14960 && node1ID == 14958)
            // {
            //     p_logger->info("Node 14960 Node 14958 : {}", l0i);
            //     overlap6058 = overlap;
            // }
            sm_L.insert(m_F02Node[node1ID], m_F02Node[node2ID]) = l0i;
        }
    }
    // if(overlap5860 != overlap6058)
    // {
    //     p_logger->info("overlap 58 60 not symmetric");
    // }
    // else
    // {
    //     p_logger->info("overlap 58 60 symmetric");
    // }
    p_logger->info("m_L calculation done");

    for (int k = 0; k < sm_L.outerSize(); ++k) 
    {
        for(Eigen::SparseMatrix<double>::InnerIterator it(sm_L, k); it; ++it) 
        {
            int row = it.row();   // 当前行
            int col = it.col();   // 当前列
            double value = it.value();  // 当前值
            if(std::fabs(sm_L.coeffRef(col, row) - value) > 1e-2)
            {
                p_logger->info("Node {} and Node {} , 12 = {}, 21 = {}", 
                                m_Node2F0[row], m_Node2F0[col], 
                                value, sm_L.coeffRef(col,row));
                outputBbox(m_Node2F0[row]);
                outputBbox(m_Node2F0[col]);
            }
        }
    }
    return true;
}

bool PoissonReconstruction::computeL2()
{
    // sm_L = Eigen::SparseMatrix<double>(m_overlap_map_map.size(), m_overlap_map_map.size());
    // // Bbox_3 overlap5860;
    // // Bbox_3 overlap6058;
    // for(auto node1 : m_overlap_map_map)
    // {
    //     NodeIndex node1ID = node1.first;
    //     for(auto node2 : node1.second)
    //     {
    //         NodeIndex node2ID = node2.first;
    //         Bbox_3 overlap = node2.second;
    //         double l0i = call0(node2ID, node1ID, overlap);
    //         // Bbox_3 overlap5860;
    //         // Bbox_3 overlap6058;
    //         // if(node2ID == 14958 && node1ID == 14960)
    //         // {
    //         //     p_logger->info("Node 14958 Node 14960 : {}", l0i);
    //         //     overlap5860 = overlap;
    //         // }
    //         // if(node2ID == 14960 && node1ID == 14958)
    //         // {
    //         //     p_logger->info("Node 14960 Node 14958 : {}", l0i);
    //         //     overlap6058 = overlap;
    //         // }
    //         if(node2ID > sm_L.rows() || node1ID > sm_L.cols()) {
    //             p_logger->error("insert location larger than sm_L size");
    //         }
    //         sm_L.insert(node2ID, node1ID) = l0i;
    //     }
    // }
    // // if(overlap5860 != overlap6058)
    // // {
    // //     p_logger->info("overlap 58 60 not symmetric");
    // // }
    // // else
    // // {
    // //     p_logger->info("overlap 58 60 symmetric");
    // // }
    // p_logger->info("m_L calculation done");

    sm_L = Eigen::SparseMatrix<double>(m_overlap_map_map.size(), m_overlap_map_map.size());
    std::vector<Eigen::Triplet<double>> triplets;

    // 转换为连续存储结构（如 std::vector）
    std::vector<std::pair<NodeIndex, std::unordered_map<NodeIndex, Bbox_3>>> node_vec(
        m_overlap_map_map.begin(), m_overlap_map_map.end()
    );

    // 预统计非零元素数量（可选）
    size_t total_nnz = 0;
    for (const auto& node1 : node_vec) {
        total_nnz += node1.second.size();
    }
    sm_L.reserve(total_nnz);

    // 并行填充 Triplet
    #pragma omp parallel
    {
        std::vector<Eigen::Triplet<double>> local_triplets;
        #pragma omp for nowait
        for (size_t i = 0; i < node_vec.size(); ++i) {
            auto node1 = node_vec[i];
            for (auto node2 : node1.second) {
                double l0i = call0(node2.first, node1.first, node2.second);
                local_triplets.emplace_back(node2.first, node1.first, l0i);
            }
        }
        #pragma omp critical
        triplets.insert(triplets.end(), local_triplets.begin(), local_triplets.end());
    }

    // 批量构建矩阵
    sm_L.setFromTriplets(triplets.begin(), triplets.end());
    p_logger->info("m_L calculation done");


    // for (int k = 0; k < sm_L.outerSize(); ++k) 
    // {
    //     for(Eigen::SparseMatrix<double>::InnerIterator it(sm_L, k); it; ++it) 
    //     {
    //         int row = it.row();   // 当前行
    //         int col = it.col();   // 当前列
    //         double value = it.value();  // 当前值
    //         if(std::fabs(sm_L.coeffRef(col, row) - value) > 1e-2)
    //         {
    //             p_logger->info("Node {} and Node {} , 12 = {}, 21 = {}", 
    //                             m_Node2F0[row], m_Node2F0[col], 
    //                             value, sm_L.coeffRef(col,row));
    //             outputBbox(m_Node2F0[row]);
    //             outputBbox(m_Node2F0[col]);
    //         }
    //     }
    // }
    return true;
}

bool PoissonReconstruction::computeX()
{
    /* check matrix */
    sm_L.makeCompressed();
    if(sm_L.rows() != m_V.rows()) {
        p_logger->error("m_V size != sm_L rows");
    }

    // m_X = m_L.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(m_V);
    using SpMat = Eigen::SparseMatrix<double>;

    // Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int>> QR(sm_L);
    // QR.compute(sm_L);
    // // 检查分解是否成功
    // if (QR.info() != Eigen::Success) {
    //     p_logger->error("Decomposition failed!");
    // }
    // p_logger->info("QR computer done");
    // m_X = QR.solve(m_V);
    // // 检查解的合理性
    // if (QR.info() != Eigen::Success) {
    //     p_logger->error("Solving failed!");
    // }

    Eigen::SparseLU<SpMat, Eigen::COLAMDOrdering<int>> LU;
    LU.analyzePattern(sm_L);
    if (LU.info() != Eigen::Success) {
        p_logger->error("AnalyzePattern failed!");
    }
    LU.factorize(sm_L);
    if (LU.info() != Eigen::Success) {
        p_logger->error("Factorize failed!");
    }
    m_X = LU.solve(m_V);
    if (LU.info() != Eigen::Success) {
        p_logger->error("Solving failed!");
    }

    // Eigen::SimplicialCholesky<SpMat> chol(sm_L);
    // m_X = chol.solve(m_V);
    // Eigen::BiCGSTAB<SpMat> solver;
    // solver.compute(sm_L);
    // m_X = solver.solve(m_V);
    p_logger->info("m_X calculation done");
    return true;
}

bool PoissonReconstruction::computeScaleC()
{
    m_scale_coefficient = 0.0;
    for(int i=0;i<m_points_set.size();++i) {
        Point& point = m_points_set.point(i);
        m_scale_coefficient += calIndicator(point);
    }
    m_scale_coefficient /= m_points_set.size();
    p_logger->info("Scale Coefficient : {}", m_scale_coefficient);
    return true;
}

bool PoissonReconstruction::marchingCubes()
{
    std::array<std::array<int,16>, 256> triTable = m_triTable;
    std::array<std::array<int,2>, 12>& edgeTable = m_edgeTable;
    /**
     * @details marching cubes vertices and edge define
     *   y                  1---e9-----5
     *    |                /|         /|
     *    |             e0  e1      e4 e5 
     *    |              /  |       /  |
     *    2----x        0---e8-----4   |
     *   /              |   2---e10|---6
     *  /               |   /      |  /
     * z                |  e2      | e6
     *                  e3 /      e7 /
     *                  | /        |/
     *                  3---e11---7    
     */   
    /**
     * @brief Octree child node index
     *     2-------3
     *    /|      /|
     *   / |     / |
     *  /  |    /  |
     * 6-------7   |
     * |   0---|---1
     * |  /    |  /
     * | /     | /
     * |/      |/
     * 4-------5
     * @details
     * enum Child {
     *      LEFT_BOTTOM_BACK,
     *      RIGHT_BOTTOM_BACK,
     *      LEFT_TOP_BACK,
     *      RIGHT_TOP_BACK,
     *      LEFT_BOTTOM_FRONT,
     *      RIGHT_BOTTOM_FRONT,
     *      LEFT_TOP_FRONT,
     *      RIGHT_TOP_FRONT
     *  };
     */ 
    p_logger->info("Marching Cubes ------ PreSplit Nodes");
    for(auto node : p_octree->traverse<LeavesTraversal>()) {
        preSplitRecursively(node);
    }
    p_logger->info("Marching Cubes ------ PreSplit Nodes done");

    p_logger->info("Marching Cubes ------ Triangles calculation");
    for(auto node : p_octree->traverse<LeavesTraversal>()) {
        if (m_indices[node].to_ulong() == 0)
            continue;
        OctreeBbox3 obbox = p_octree->bbox(node);
        std::vector<Point> nodeVertices({
            {obbox.xmin(), obbox.ymax(),obbox.zmax()},
            {obbox.xmin(), obbox.ymax(),obbox.zmin()},
            {obbox.xmin(), obbox.ymin(),obbox.zmin()},
            {obbox.xmin(), obbox.ymin(),obbox.zmax()},
            {obbox.xmax(), obbox.ymax(),obbox.zmax()},
            {obbox.xmax(), obbox.ymax(),obbox.zmin()},
            {obbox.xmax(), obbox.ymin(),obbox.zmin()},
            {obbox.xmax(), obbox.ymin(),obbox.zmax()}
        });
        std::array<int, 16>& allEdgeIDs = triTable[m_indices[node].to_ulong()];
        std::array<double,8>& nodeIndicators = m_node_indicators[node];
        std::vector<std::vector<int>> tri2edge;
        for(int i=0;i<16;i+=3) {
            if(allEdgeIDs[i] == -1) 
                continue;
            std::vector<int> tri({allEdgeIDs[i], allEdgeIDs[i+1], allEdgeIDs[i+2]});
            tri2edge.push_back(tri);
        }
        for(int i=0;i<tri2edge.size();++i) {
            std::vector<int>& edges = tri2edge[i];
            std::vector<int> triangle;
            for(int j=0;j<edges.size();++j) {
                int node1ID = edgeTable[edges[j]][0];
                int node2ID = edgeTable[edges[j]][1];
                double node1Indicator = nodeIndicators[node1ID];
                double node2Indicator = nodeIndicators[node2ID];
                double t = (m_scale_coefficient - node1Indicator) / (node2Indicator - node1Indicator);
                Eigen::Vector3d p1(nodeVertices[node1ID].x(), nodeVertices[node1ID].y(), nodeVertices[node1ID].z());
                Eigen::Vector3d p2(nodeVertices[node2ID].x(), nodeVertices[node2ID].y(), nodeVertices[node2ID].z());
                Eigen::Vector3d p0 = p1 + t * (p2 - p1);
                m_polygon_points.push_back({p0.x(), p0.y(), p0.z()});
                triangle.push_back(m_polygon_points.size()-1);
            }
            m_polygon_soup.push_back(triangle);
        }
    }

    // for(auto node : p_octree->traverse<LeavesTraversal>()) {
    //     splitRecursively(node);
    // }
    
    // p_logger->info("Marching Cubes ------ Triangles calculation");
    // for(auto node : p_octree->traverse<LeavesTraversal>()) {
    //     if (m_indices[node].to_ulong() == 0)
    //         continue;
    //     OctreeBbox3 obbox = p_octree->bbox(node);
    //     std::vector<Point> nodeVertices({
    //         {obbox.xmin(), obbox.ymax(),obbox.zmax()},
    //         {obbox.xmin(), obbox.ymax(),obbox.zmin()},
    //         {obbox.xmin(), obbox.ymin(),obbox.zmin()},
    //         {obbox.xmin(), obbox.ymin(),obbox.zmax()},
    //         {obbox.xmax(), obbox.ymax(),obbox.zmax()},
    //         {obbox.xmax(), obbox.ymax(),obbox.zmin()},
    //         {obbox.xmax(), obbox.ymin(),obbox.zmin()},
    //         {obbox.xmax(), obbox.ymin(),obbox.zmax()}
    //     });
    //     std::array<int, 16>& allEdgeIDs = triTable[m_indices[node].to_ulong()];
    //     std::array<double,8>& nodeIndicators = m_node_indicators[node];
    //     std::vector<std::vector<int>> tri2edge;
    //     for(int i=0;i<16;i+=3) {
    //         if(allEdgeIDs[i] == -1) 
    //             continue;
    //         std::vector<int> tri({allEdgeIDs[i], allEdgeIDs[i+1], allEdgeIDs[i+2]});
    //         tri2edge.push_back(tri);
    //     }
    //     for(int i=0;i<tri2edge.size();++i) {
    //         std::vector<int>& edges = tri2edge[i];
    //         std::vector<int> triangle;
    //         for(int j=0;j<edges.size();++j) {
    //             int node1ID = edgeTable[edges[j]][0];
    //             int node2ID = edgeTable[edges[j]][1];
    //             double node1Indicator = nodeIndicators[node1ID];
    //             double node2Indicator = nodeIndicators[node2ID];
    //             double t = (m_scale_coefficient - node1Indicator) / (node2Indicator - node1Indicator);
    //             Eigen::Vector3d p1(nodeVertices[node1ID].x(), nodeVertices[node1ID].y(), nodeVertices[node1ID].z());
    //             Eigen::Vector3d p2(nodeVertices[node2ID].x(), nodeVertices[node2ID].y(), nodeVertices[node2ID].z());
    //             Eigen::Vector3d p0 = p1 + t * (p2 - p1);
    //             m_polygon_points.push_back({p0.x(), p0.y(), p0.z()});
    //             triangle.push_back(m_polygon_points.size()-1);
    //         }
    //         m_polygon_soup.push_back(triangle);
    //     }
    // }
    p_logger->info("Marching Cubes ------ done");
    return true;
}

bool PoissonReconstruction::preSplitRecursively(NodeIndex node)
{
    OctreeBbox3 obbox = p_octree->bbox(node);
    std::vector<Point> nodeVertices({
        {obbox.xmin(), obbox.ymax(),obbox.zmax()},
        {obbox.xmin(), obbox.ymax(),obbox.zmin()},
        {obbox.xmin(), obbox.ymin(),obbox.zmin()},
        {obbox.xmin(), obbox.ymin(),obbox.zmax()},
        {obbox.xmax(), obbox.ymax(),obbox.zmax()},
        {obbox.xmax(), obbox.ymax(),obbox.zmin()},
        {obbox.xmax(), obbox.ymin(),obbox.zmin()},
        {obbox.xmax(), obbox.ymin(),obbox.zmax()}
    });
    calNodeVerIndicatorAndIndex(node, nodeVertices);
    if(m_indices[node].to_ulong() == 0)
        return false;
    if(p_octree->depth(node) < p_octree->depth()) {
        p_octree->split(node);
        for(int i=0;i<8;++i) 
            preSplitRecursively(p_octree->child(node,i));
    }
    return true;
}

void PoissonReconstruction::splitRecursively(NodeIndex node)
{
    std::array<std::array<int,16>, 256> triTable = m_triTable;
    std::array<std::array<int,2>, 12>& edgeTable = m_edgeTable;
    OctreeBbox3 obbox = p_octree->bbox(node);
    std::vector<Point> nodeVertices({
        {obbox.xmin(), obbox.ymax(),obbox.zmax()},
        {obbox.xmin(), obbox.ymax(),obbox.zmin()},
        {obbox.xmin(), obbox.ymin(),obbox.zmin()},
        {obbox.xmin(), obbox.ymin(),obbox.zmax()},
        {obbox.xmax(), obbox.ymax(),obbox.zmax()},
        {obbox.xmax(), obbox.ymax(),obbox.zmin()},
        {obbox.xmax(), obbox.ymin(),obbox.zmin()},
        {obbox.xmax(), obbox.ymin(),obbox.zmax()}
    });
    calNodeVerIndicatorAndIndex(node, nodeVertices);
    std::array<int, 16>& allEdgeIDs = triTable[m_indices[node].to_ulong()];
    std::vector<int> edges;
    std::vector<Point> triP;
    bool needSplit = false;
    for(int i=0;i<16 && !needSplit;i+=3) {
        if(allEdgeIDs[i] == -1) 
            continue;
        for(int j=0;j<3 && !needSplit;j++) {
            edges.push_back(allEdgeIDs[i+j]);
            Segment edge(nodeVertices[edgeTable[edges.back()][0]], nodeVertices[edgeTable[edges.back()][1]]);
            auto zeroCrossOpt = finestZeroCrossing(edge, node);
            Point ipoint;
            if(auto* p = std::get_if<Point>(&zeroCrossOpt)) {
                ipoint = *p;
                triP.push_back(ipoint);
            } else {
                int* size = std::get_if<int>(&zeroCrossOpt);
                if(size == 0) {
                    ipoint = *interPolatePoint(edge);
                    triP.push_back(ipoint);
                }else {
                    p_logger->info("Need split");
                    needSplit = true;
                }
            }
        }
    }
    if(needSplit){
        p_octree->split(node);
        for(int i=0;i<8;++i) {
            splitRecursively(p_octree->child(node,i));
        }
    } else {
        for(int i = 0; i < triP.size(); i+=3) {
            std::vector<int> triID;
            m_polygon_points.push_back(triP[i]);
            m_polygon_points.push_back(triP[i+1]);
            m_polygon_points.push_back(triP[i+2]);
            m_polygon_soup.push_back({(int)m_polygon_points.size()-3, 
                                        (int)m_polygon_points.size()-2, 
                                        (int)m_polygon_points.size()-1});
        }
    }
}

void PoissonReconstruction::validation()
{
    /**
     * generate random points
     */
    OctreeBbox3 obbox = p_octree->bbox(p_octree->root());
    Bbox_3 bbox = obbox.bbox();
    double lx = bbox.xmax() - bbox.xmin(),
           ly = bbox.ymax() - bbox.ymin(),
           lz = bbox.zmax() - bbox.zmin();
    std::default_random_engine generator;
    std::uniform_int_distribution<int> uniform_distribution(0, 10000);
    std::vector<Point> generate_points(10000);
    std::vector<CGAL::Color> colors(10000);
    #pragma omp parallel 
    {
        #pragma omp for   
        for(int i=0;i<10000;++i)
        {
            int rx = uniform_distribution(generator),
                ry = uniform_distribution(generator),
                rz = uniform_distribution(generator);
            double x = rx / 10000.0 * lx + bbox.xmin();
            double y = ry / 10000.0 * ly + bbox.ymin();
            double z = rz / 10000.0 * lz + bbox.zmin();
            generate_points[i]=Point(x,y,z);
            double r = 0.0, g = 0.0, b = 0.0;
            if(calIndicator(generate_points.back()) - m_scale_coefficient > 0.0) {
                r = 255.0;
            }
            else {
                b = 255.0;
            }
            colors[i] = CGAL::Color(r, g, b);
        }
    }
    
    std::string filename("../output/validation.ply");
    std::ofstream stream(filename);
    if(!stream)
    {
        p_logger->error("Error: cannot open file {}", filename);
        return;
    }
    stream << "ply\n";
    stream << "format ascii 1.0\n";
    stream << "element vertex " << generate_points.size() << "\n";
    stream << "property float x\n";
    stream << "property float y\n";
    stream << "property float z\n";
    stream << "property uchar red"   << std::endl;
    stream << "property uchar green" << std::endl;
    stream << "property uchar blue"  << std::endl;
    stream << "element face " << 0   << std::endl;
    stream << "property list uchar int vertex_index" << std::endl;
    stream << "end_header" <<  std::endl;
    // Vertices
    for (size_t i = 0; i < generate_points.size(); ++i) {
            stream  << generate_points[i].x() << " " 
                    << generate_points[i].y() << " " 
                    << generate_points[i].z() << " " 
                    << (int)colors[i].r() << " "
                    << (int)colors[i].g() << " " 
                    << (int)colors[i].b() << std::endl;
    }
    stream << std::endl;
    stream.close();
}

void PoissonReconstruction::Execute()
{
    

    if(!loadPointsWithNormal())
        return ;
    else
        p_logger->info("Successfully loaded points from {}.", m_filename);
    CGALPSR();

    if(!reverseNormal())
        return;
    else
        p_logger->info("Successfully reversenormal.");
    if(!buildOctree())
        return;
    else
        p_logger->info("Successfully build octree.");
    OutputOctree("../output/octree.ply");

    // if(!buildVectorFieldD())
    //     return;
    // else
    //     p_logger->info("Successfully build vector field.");
    if(!buildVectorField())
        return;
    else
        p_logger->info("Successfully build vector field.");
    OutputOctree("../output/octree_after.ply");
    outputLeavesBbox();

    computeV2();
    // std::ofstream streamV("../output/m_V2.txt");
    // streamV << m_V << std::endl;
    // streamV.close();
    computeL2();
    // std::ofstream streamL("../output/sm_L2.txt");
    // streamL << sm_L << std::endl;
    // streamL.close();
    computeX();
    // std::ofstream streamX("../output/m_X2.txt");
    // streamX << m_X << std::endl;
    // streamX.close();
    
    computeScaleC();
    marchingCubes();
    outputMarchingCubesResuilt();
    validation();
    checkSymmetric();

    p_logger->info("done");
    // computeVD();
    // std::ofstream streamV("../output/m_VD.txt");
    // streamV << m_V << std::endl;
    // computeLD();
    // std::ofstream streamL("../output/sm_LD.txt");
    // streamL << sm_L << std::endl;
    // computeX();
    // std::ofstream streamX("../output/m_X.txt");
    // streamX << m_X << std::endl;
    // validation();

    // if(!buildVectorField())
    //     return;
    // else
    //     p_logger->info("Successfully build vector field.");

    // OutputOctree("../output/octree_after.ply");

    // computeV();
    // std::ofstream streamV("../output/m_V.txt");
    // streamV << m_V << std::endl;
    // computeL();
    // std::ofstream streamL("../output/sm_L.txt");
    // streamL << sm_L << std::endl;
    // computeX();
    // std::ofstream streamX("../output/m_X.txt");
    // streamX << m_X << std::endl;
    // validation();
}

void PoissonReconstruction::SetParameters(const std::vector<std::any>& parameters)
{

    m_filename = std::any_cast<std::string>(parameters[0]);
    m_co_bbox = std::any_cast<double>(parameters[1]);
}

bool PoissonReconstruction::checkSymmetric()
{
    std::cout << "快速对称性检查: ";
    if (sm_L.isApprox(sm_L.transpose())) {
        std::cout << "矩阵是对称的" << std::endl;
    } else {
        std::cout << "矩阵不对称" << std::endl;
    }

    if (sm_L.rows() != sm_L.cols()) {
        p_logger->info("非方阵");
        return false; // 非方阵直接返回
    } 

    // // 遍历每个非零元素
    // for (int col = 0; col < sm_L.outerSize(); ++col) {
    //     for (Eigen::SparseMatrix<double>::InnerIterator it(sm_L, col); it; ++it) {
    //         int row = it.row();
    //         int current_col = col; // Eigen中存储为压缩列格式     
    //         // 仅处理上三角部分避免重复检查
    //         if (row > current_col) continue;     
    //         // 获取当前元素值
    //         double val = it.value();    
    //         // 在对称位置查找对应元素
    //         bool found = false;
    //         for (Eigen::SparseMatrix<double>::InnerIterator sym_it(sm_L, row); sym_it; ++sym_it) {
    //             if (sym_it.row() == current_col && sym_it.col() == row) {
    //                 double sym_val = sym_it.value();
    //                 if (std::abs(val - sym_val) > 1e-12) {
    //                     p_logger->info("row {} col {} value: {}  {}", row , current_col, val, sym_val);
    //                     return false; // 数值不匹配
    //                 }
    //                 found = true;
    //                 break;
    //             }
    //         }      
    //         // 非对角元素必须存在对称项
    //         if (row != current_col && !found) {
    //             p_logger->info("row {} col {} 存在不对称存储", row , current_col);
    //             return false;
    //         } 
    //     }
    // }
    // p_logger->info("每个元素对比并未发现不对称");

    bool not_symmetric = false;
    // 遍历每个非零元素
    for (int col = 0; col < sm_L.outerSize(); ++col) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(sm_L, col); it; ++it) {
            int row = it.row();
            int current_col = col;

            // 仅处理上三角部分避免重复检查
            if (row > current_col) continue;

            // 获取当前元素值
            double val = it.value();

            // 获取对称位置的值
            double sym_val = sm_L.coeff(row, current_col);

            // 检查对称性
            if (std::abs(val - sym_val) > 1e-12 * std::max(std::abs(val), std::abs(sym_val))) {
                p_logger->info("row {} col {} value: {}  {}", row, current_col, val, sym_val);
                // return false; // 数值不匹配
                not_symmetric = true;
            }
        }
    }
    if(!not_symmetric)
        p_logger->info("每个元素对比并未发现不对称");
    else
        p_logger->info("发现不对称");
    p_logger->info("done check");
    return true;
}

void PoissonReconstruction::CGALPSR()
{
    std::vector<Pwn> points;
    for(int i=0;i<m_PWNs.size();++i) {
        points.push_back(m_PWNs[i]);
    }
    Polyhedron output_mesh;
    double average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>
                                (points, 6, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>()));

    if (CGAL::poisson_surface_reconstruction_delaunay
      (points.begin(), points.end(),
       CGAL::First_of_pair_property_map<Pwn>(),
       CGAL::Second_of_pair_property_map<Pwn>(),
       output_mesh, average_spacing))
    {
        std::ofstream out("../output/poisson.off");
        out << output_mesh;
    }
    else {
        p_logger->error("Failed CGALPSR");
    }
}   

/// @}
/// @name Access
/// @{
/// @}
/// @name Inquiry
/// @{
/// @}
/// @name Input and Output
/// @{

bool PoissonReconstruction::loadPointsWithNormal()
{
    std::string path_point_cloud = m_filename;
    std::string path_point_cloud_extension = boost::filesystem::path(path_point_cloud).extension().string();
    if(!boost::filesystem::exists(path_point_cloud)){
        p_logger->error("File {} does not exist",path_point_cloud);
        return false;
    }
    p_logger->info("Load points from {}", m_filename);

    m_PWNs.clear();
    if(path_point_cloud_extension != ".ply")
    {
        p_logger->error("Only support ply file!");
        return false;
    }

    std::ifstream streamb(m_filename);
    int line = 0;
    std::string s;
    while (streamb && line < 2) {
        if (!getline(streamb, s)) break;
        line++;
    }
    streamb.close();

    if (s == "format ascii 1.0") {
        std::ifstream stream(m_filename);
        if (!stream || !CGAL::IO::read_PLY_with_properties(stream, std::back_inserter(m_PWNs),
                                                                   CGAL::IO::make_ply_point_reader(Point_map()),
                                                                   CGAL::IO::make_ply_normal_reader(Normal_map()))) {
            p_logger->info("wrong1");
            return false;
        }
    }
    else {
        std::ifstream stream(m_filename, std::ios_base::binary);
        if (!stream || !CGAL::IO::read_PLY_with_properties(stream, std::back_inserter(m_PWNs),
                                                           CGAL::IO::make_ply_point_reader(Point_map()),
                                                           CGAL::IO::make_ply_normal_reader(Normal_map()))) {
            p_logger->info("wrong2");
            return false;
        }
    }
    
    if(m_PWNs[0].second == Vector(0,0,0))
    {
        p_logger->warn("Normal is not oriented, orienting normal");
        return false;
    }

    return true;
}

void PoissonReconstruction::OutputOctree(std::string filename)
{
    std::vector<Point> vertices; // 用于存储所有顶点
    std::vector<std::tuple<std::size_t, std::size_t, std::size_t>> edges; // 用于存储所有边（索引对）
    // 遍历八叉树的叶节点
    for (auto node : p_octree->traverse<LeavesTraversal>())
    {
        // 获取节点的包围盒
        OctreeBbox3 bbox = p_octree->bbox(node);
        // 计算包围盒的 8 个顶点
        Point v0(bbox.xmin(), bbox.ymin(), bbox.zmin());
        Point v1(bbox.xmax(), bbox.ymin(), bbox.zmin());
        Point v2(bbox.xmax(), bbox.ymax(), bbox.zmin());
        Point v3(bbox.xmin(), bbox.ymax(), bbox.zmin());
        Point v4(bbox.xmin(), bbox.ymin(), bbox.zmax());
        Point v5(bbox.xmax(), bbox.ymin(), bbox.zmax());
        Point v6(bbox.xmax(), bbox.ymax(), bbox.zmax());
        Point v7(bbox.xmin(), bbox.ymax(), bbox.zmax());
        // 获取当前顶点的起始索引
        std::size_t base_index = vertices.size();
        // 添加顶点到顶点列表
        vertices.push_back(v0);
        vertices.push_back(v1);
        vertices.push_back(v2);
        vertices.push_back(v3);
        vertices.push_back(v4);
        vertices.push_back(v5);
        vertices.push_back(v6);
        vertices.push_back(v7);
        // 添加边到边列表（每条边由两个顶点索引组成）
        edges.emplace_back(base_index + 0, base_index + 1, base_index + 1); // v0 -> v1
        edges.emplace_back(base_index + 1, base_index + 2, base_index + 2); // v1 -> v2
        edges.emplace_back(base_index + 2, base_index + 3, base_index + 3); // v2 -> v3
        edges.emplace_back(base_index + 3, base_index + 0, base_index + 0); // v3 -> v0
        edges.emplace_back(base_index + 4, base_index + 5, base_index + 5); // v4 -> v5
        edges.emplace_back(base_index + 5, base_index + 6, base_index + 6); // v5 -> v6
        edges.emplace_back(base_index + 6, base_index + 7, base_index + 7); // v6 -> v7
        edges.emplace_back(base_index + 7, base_index + 4, base_index + 4); // v7 -> v4
        edges.emplace_back(base_index + 0, base_index + 4, base_index + 4); // v0 -> v4
        edges.emplace_back(base_index + 1, base_index + 5, base_index + 5); // v1 -> v5
        edges.emplace_back(base_index + 2, base_index + 6, base_index + 6); // v2 -> v6
        edges.emplace_back(base_index + 3, base_index + 7, base_index + 7); // v3 -> v7
    }
    // 将顶点和边写入 PLY 文件
    // std::string filename("../output/octree.ply");
    std::ofstream out(filename);
    if (!out)
    {
        p_logger->error("Error: cannot open file {}", filename);
        return;
    }
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vertices.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "element edge " << edges.size() << "\n";
    out << "property int vertex1\n";
    out << "property int vertex2\n";
    out << "end_header\n";
    // 写入顶点
    for (const auto& vertex : vertices)
        out << vertex.x() << " " << vertex.y() << " " << vertex.z() << "\n";
    // 写入边
    for (const auto& edge : edges)
        out << std::get<0>(edge) << " " << std::get<1>(edge) << "\n";
        // out << std::get<0>(edge) << " " << std::get<1>(edge) << " " << std::get<2>(edge) << "\n";
    out.close();
    p_logger->info("Successfully exported octree bounding boxes to {}", filename);
}

void PoissonReconstruction::outputBbox(NodeIndex iNode)
{
    std::string filename = "../output/checkbbox/checkbbox_node_" + std::to_string(iNode) + ".ply";
    std::ofstream out(filename);
    if (!out)
    {
        p_logger->error("Error: cannot open file {}", filename);
        return;
    }
    std::vector<Point> vertices; // 用于存储所有顶点
    std::vector<std::tuple<std::size_t, std::size_t>> edges; // 用于存储所有边（索引对）
    std::vector<CGAL::Color> colors;

    NodeIndex node = iNode;
    // 为当前节点的 8 个顶点添加颜色
    CGAL::Color node_color = CGAL::Color(255, 0, 0);
    for (int i = 0; i < 8; ++i)
        colors.push_back(node_color);

    // 获取节点的包围盒
    OctreeBbox3 bbox = p_octree->bbox(node);

    // 计算包围盒的 8 个顶点
    Point v0(bbox.xmin(), bbox.ymin(), bbox.zmin());
    Point v1(bbox.xmax(), bbox.ymin(), bbox.zmin());
    Point v2(bbox.xmax(), bbox.ymax(), bbox.zmin());
    Point v3(bbox.xmin(), bbox.ymax(), bbox.zmin());
    Point v4(bbox.xmin(), bbox.ymin(), bbox.zmax());
    Point v5(bbox.xmax(), bbox.ymin(), bbox.zmax());
    Point v6(bbox.xmax(), bbox.ymax(), bbox.zmax());
    Point v7(bbox.xmin(), bbox.ymax(), bbox.zmax());

    // 获取当前顶点的起始索引
    std::size_t base_index = vertices.size();

    // 添加顶点到顶点列表
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
    vertices.push_back(v4);
    vertices.push_back(v5);
    vertices.push_back(v6);
    vertices.push_back(v7);

    // 添加边到边列表（每条边由两个顶点索引组成）
    edges.emplace_back(base_index + 0, base_index + 1); // v0 -> v1
    edges.emplace_back(base_index + 1, base_index + 2); // v1 -> v2
    edges.emplace_back(base_index + 2, base_index + 3); // v2 -> v3
    edges.emplace_back(base_index + 3, base_index + 0); // v3 -> v0

    edges.emplace_back(base_index + 4, base_index + 5); // v4 -> v5
    edges.emplace_back(base_index + 5, base_index + 6); // v5 -> v6
    edges.emplace_back(base_index + 6, base_index + 7); // v6 -> v7
    edges.emplace_back(base_index + 7, base_index + 4); // v7 -> v4

    edges.emplace_back(base_index + 0, base_index + 4); // v0 -> v4
    edges.emplace_back(base_index + 1, base_index + 5); // v1 -> v5
    edges.emplace_back(base_index + 2, base_index + 6); // v2 -> v6
    edges.emplace_back(base_index + 3, base_index + 7); // v3 -> v7
    // 将顶点和边写入 PLY 文件
    // std::string filename("../output/octree.ply");
    
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vertices.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "element edge " << edges.size() << "\n";
    out << "property int vertex1\n";
    out << "property int vertex2\n";
    out << "end_header\n";
    out.flush(); // 确保头部内容写入文件
    // 写入顶点
    for (int i = 0; i < vertices.size(); ++i)
    {
        out << (float)vertices[i].x() << " "
            << (float)vertices[i].y() << " "
            << (float)vertices[i].z() << " "
            << (int)colors[i].r() << " "
            << (int)colors[i].g() << " "
            << (int)colors[i].b() << "\n";
    }

    // 写入边
    for (const auto& edge : edges)
    {
        out  << std::get<0>(edge) << " " << std::get<1>(edge) << "\n";
    }

    out.close();
     
}

void PoissonReconstruction::output3Bbox(NodeIndex iNode, NodeIndex iNode2)
{
    std::string filedir = "../output/checkOverlap3Bbox";
    boost::filesystem::create_directory(filedir);
    std::string filename = "../output/checkOverlap3Bbox/checkbbox_node_" + std::to_string(iNode) + "_" + std::to_string(iNode2) + ".ply";
    std::ofstream out(filename);
    if (!out)
    {
        p_logger->error("Error: cannot open file {}", filename);
        return;
    }
    std::vector<Point> vertices; // 用于存储所有顶点
    std::vector<std::tuple<std::size_t, std::size_t>> edges; // 用于存储所有边（索引对）
    std::vector<CGAL::Color> colors;

    NodeIndex node = iNode;
    // 为当前节点的 8 个顶点添加颜色
    CGAL::Color node_color = CGAL::Color(255, 0, 0);
    for (int i = 0; i < 8; ++i)
        colors.push_back(node_color);
    CGAL::Color node_color2 = CGAL::Color(255, 255, 0);
    for (int i = 0; i < 8; ++i)
        colors.push_back(node_color2);

    // 获取节点的包围盒
    Bbox_3 bbox1 = p_octree->bbox(node).bbox();
    bbox1.scale(3.0);

    // 计算包围盒的 8 个顶点
    Point v0(bbox1.xmin(), bbox1.ymin(), bbox1.zmin());
    Point v1(bbox1.xmax(), bbox1.ymin(), bbox1.zmin());
    Point v2(bbox1.xmax(), bbox1.ymax(), bbox1.zmin());
    Point v3(bbox1.xmin(), bbox1.ymax(), bbox1.zmin());
    Point v4(bbox1.xmin(), bbox1.ymin(), bbox1.zmax());
    Point v5(bbox1.xmax(), bbox1.ymin(), bbox1.zmax());
    Point v6(bbox1.xmax(), bbox1.ymax(), bbox1.zmax());
    Point v7(bbox1.xmin(), bbox1.ymax(), bbox1.zmax());

    // 获取当前顶点的起始索引
    std::size_t base_index = vertices.size();

    // 添加顶点到顶点列表
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
    vertices.push_back(v4);
    vertices.push_back(v5);
    vertices.push_back(v6);
    vertices.push_back(v7);

    // 添加边到边列表（每条边由两个顶点索引组成）
    edges.emplace_back(base_index + 0, base_index + 1); // v0 -> v1
    edges.emplace_back(base_index + 1, base_index + 2); // v1 -> v2
    edges.emplace_back(base_index + 2, base_index + 3); // v2 -> v3
    edges.emplace_back(base_index + 3, base_index + 0); // v3 -> v0

    edges.emplace_back(base_index + 4, base_index + 5); // v4 -> v5
    edges.emplace_back(base_index + 5, base_index + 6); // v5 -> v6
    edges.emplace_back(base_index + 6, base_index + 7); // v6 -> v7
    edges.emplace_back(base_index + 7, base_index + 4); // v7 -> v4

    edges.emplace_back(base_index + 0, base_index + 4); // v0 -> v4
    edges.emplace_back(base_index + 1, base_index + 5); // v1 -> v5
    edges.emplace_back(base_index + 2, base_index + 6); // v2 -> v6
    edges.emplace_back(base_index + 3, base_index + 7); // v3 -> v7


    node = iNode2;
    // 获取节点的包围盒
    bbox1 = p_octree->bbox(node).bbox();
    bbox1.scale(3.0);

    // 计算包围盒的 8 个顶点
    v0 = Point(bbox1.xmin(), bbox1.ymin(), bbox1.zmin());
    v1 = Point(bbox1.xmax(), bbox1.ymin(), bbox1.zmin());
    v2 = Point(bbox1.xmax(), bbox1.ymax(), bbox1.zmin());
    v3 = Point(bbox1.xmin(), bbox1.ymax(), bbox1.zmin());
    v4 = Point(bbox1.xmin(), bbox1.ymin(), bbox1.zmax());
    v5 = Point(bbox1.xmax(), bbox1.ymin(), bbox1.zmax());
    v6 = Point(bbox1.xmax(), bbox1.ymax(), bbox1.zmax());
    v7 = Point(bbox1.xmin(), bbox1.ymax(), bbox1.zmax());

    // 获取当前顶点的起始索引
    base_index = vertices.size();

    // 添加顶点到顶点列表
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
    vertices.push_back(v4);
    vertices.push_back(v5);
    vertices.push_back(v6);
    vertices.push_back(v7);

    // 添加边到边列表（每条边由两个顶点索引组成）
    edges.emplace_back(base_index + 0, base_index + 1); // v0 -> v1
    edges.emplace_back(base_index + 1, base_index + 2); // v1 -> v2
    edges.emplace_back(base_index + 2, base_index + 3); // v2 -> v3
    edges.emplace_back(base_index + 3, base_index + 0); // v3 -> v0

    edges.emplace_back(base_index + 4, base_index + 5); // v4 -> v5
    edges.emplace_back(base_index + 5, base_index + 6); // v5 -> v6
    edges.emplace_back(base_index + 6, base_index + 7); // v6 -> v7
    edges.emplace_back(base_index + 7, base_index + 4); // v7 -> v4

    edges.emplace_back(base_index + 0, base_index + 4); // v0 -> v4
    edges.emplace_back(base_index + 1, base_index + 5); // v1 -> v5
    edges.emplace_back(base_index + 2, base_index + 6); // v2 -> v6
    edges.emplace_back(base_index + 3, base_index + 7); // v3 -> v7


    // 将顶点和边写入 PLY 文件
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vertices.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "element edge " << edges.size() << "\n";
    out << "property int vertex1\n";
    out << "property int vertex2\n";
    out << "end_header\n";
    out.flush(); // 确保头部内容写入文件
    // 写入顶点
    for (int i = 0; i < vertices.size(); ++i)
    {
        out << (float)vertices[i].x() << " "
            << (float)vertices[i].y() << " "
            << (float)vertices[i].z() << " "
            << (int)colors[i].r() << " "
            << (int)colors[i].g() << " "
            << (int)colors[i].b() << "\n";
    }

    // 写入边
    for (const auto& edge : edges)
    {
        out  << std::get<0>(edge) << " " << std::get<1>(edge) << "\n";
    }

    out.close();
}

void PoissonReconstruction::outputLeavesBbox()
{
    std::string filename("../output/checkbbox/BboxLeaves.ply");
    std::ofstream out(filename);
    if (!out)
    {
        p_logger->error("Error: cannot open file {}", filename);
        return;
    }
    std::vector<Point> vertices; // 用于存储所有顶点
    std::vector<std::tuple<std::size_t, std::size_t>> edges; // 用于存储所有边（索引对）
    std::vector<CGAL::Color> colors;
    for(auto node : p_octree->traverse<LeavesTraversal>())
    {
        // 为当前节点的 8 个顶点添加颜色
        CGAL::Color node_color = CGAL::Color(255.0, 0, 0);
        for (int i = 0; i < 8; ++i)
            colors.push_back(node_color);

        // 获取节点的包围盒
        OctreeBbox3 bbox = p_octree->bbox(node);

        // 计算包围盒的 8 个顶点
        Point v0(bbox.xmin(), bbox.ymin(), bbox.zmin());
        Point v1(bbox.xmax(), bbox.ymin(), bbox.zmin());
        Point v2(bbox.xmax(), bbox.ymax(), bbox.zmin());
        Point v3(bbox.xmin(), bbox.ymax(), bbox.zmin());
        Point v4(bbox.xmin(), bbox.ymin(), bbox.zmax());
        Point v5(bbox.xmax(), bbox.ymin(), bbox.zmax());
        Point v6(bbox.xmax(), bbox.ymax(), bbox.zmax());
        Point v7(bbox.xmin(), bbox.ymax(), bbox.zmax());

        // 获取当前顶点的起始索引
        std::size_t base_index = vertices.size();
        // 添加顶点到顶点列表
        vertices.push_back(v0);
        vertices.push_back(v1);
        vertices.push_back(v2);
        vertices.push_back(v3);
        vertices.push_back(v4);
        vertices.push_back(v5);
        vertices.push_back(v6);
        vertices.push_back(v7);
        // 添加边到边列表（每条边由两个顶点索引组成）
        edges.emplace_back(base_index + 0, base_index + 1); // v0 -> v1
        edges.emplace_back(base_index + 1, base_index + 2); // v1 -> v2
        edges.emplace_back(base_index + 2, base_index + 3); // v2 -> v3
        edges.emplace_back(base_index + 3, base_index + 0); // v3 -> v0
        edges.emplace_back(base_index + 4, base_index + 5); // v4 -> v5
        edges.emplace_back(base_index + 5, base_index + 6); // v5 -> v6
        edges.emplace_back(base_index + 6, base_index + 7); // v6 -> v7
        edges.emplace_back(base_index + 7, base_index + 4); // v7 -> v4
        edges.emplace_back(base_index + 0, base_index + 4); // v0 -> v4
        edges.emplace_back(base_index + 1, base_index + 5); // v1 -> v5
        edges.emplace_back(base_index + 2, base_index + 6); // v2 -> v6
        edges.emplace_back(base_index + 3, base_index + 7); // v3 -> v7
    }
    
    // 将顶点和边写入 PLY 文件
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vertices.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "element edge " << edges.size() << "\n";
    out << "property int vertex1\n";
    out << "property int vertex2\n";
    out << "end_header\n";
    out.flush(); // 确保头部内容写入文件
    // 写入顶点
    for (int i = 0; i < vertices.size(); ++i)
    {
        out << (float)vertices[i].x() << " "
            << (float)vertices[i].y() << " "
            << (float)vertices[i].z() << " "
            << (int)colors[i].r() << " "
            << (int)colors[i].g() << " "
            << (int)colors[i].b() << "\n";
    }
    // 写入边
    for (const auto& edge : edges)
    {
        out  << std::get<0>(edge) << " " << std::get<1>(edge) << "\n";
    }
    out.close();
}

void PoissonReconstruction::outputBboxD(int depth)
{
    std::string filename = "../output/checkbbox/BboxDepth_" + std::to_string(depth)+ ".ply";
    std::ofstream out(filename);
    if (!out)
    {
        p_logger->error("Error: cannot open file {}", filename);
        return;
    }
    std::vector<Point> vertices; // 用于存储所有顶点
    std::vector<std::tuple<std::size_t, std::size_t>> edges; // 用于存储所有边（索引对）
    std::vector<CGAL::Color> colors;
    for(auto node : p_octree->traverse<LevelTraversal>(depth))
    {
        // 为当前节点的 8 个顶点添加颜色
        CGAL::Color node_color = CGAL::Color(1.0 * depth / p_octree->depth()*255.0, 0, 0);
        for (int i = 0; i < 8; ++i)
            colors.push_back(node_color);

        // 获取节点的包围盒
        OctreeBbox3 bbox = p_octree->bbox(node);

        // 计算包围盒的 8 个顶点
        Point v0(bbox.xmin(), bbox.ymin(), bbox.zmin());
        Point v1(bbox.xmax(), bbox.ymin(), bbox.zmin());
        Point v2(bbox.xmax(), bbox.ymax(), bbox.zmin());
        Point v3(bbox.xmin(), bbox.ymax(), bbox.zmin());
        Point v4(bbox.xmin(), bbox.ymin(), bbox.zmax());
        Point v5(bbox.xmax(), bbox.ymin(), bbox.zmax());
        Point v6(bbox.xmax(), bbox.ymax(), bbox.zmax());
        Point v7(bbox.xmin(), bbox.ymax(), bbox.zmax());

        // 获取当前顶点的起始索引
        std::size_t base_index = vertices.size();
        // 添加顶点到顶点列表
        vertices.push_back(v0);
        vertices.push_back(v1);
        vertices.push_back(v2);
        vertices.push_back(v3);
        vertices.push_back(v4);
        vertices.push_back(v5);
        vertices.push_back(v6);
        vertices.push_back(v7);
        // 添加边到边列表（每条边由两个顶点索引组成）
        edges.emplace_back(base_index + 0, base_index + 1); // v0 -> v1
        edges.emplace_back(base_index + 1, base_index + 2); // v1 -> v2
        edges.emplace_back(base_index + 2, base_index + 3); // v2 -> v3
        edges.emplace_back(base_index + 3, base_index + 0); // v3 -> v0
        edges.emplace_back(base_index + 4, base_index + 5); // v4 -> v5
        edges.emplace_back(base_index + 5, base_index + 6); // v5 -> v6
        edges.emplace_back(base_index + 6, base_index + 7); // v6 -> v7
        edges.emplace_back(base_index + 7, base_index + 4); // v7 -> v4
        edges.emplace_back(base_index + 0, base_index + 4); // v0 -> v4
        edges.emplace_back(base_index + 1, base_index + 5); // v1 -> v5
        edges.emplace_back(base_index + 2, base_index + 6); // v2 -> v6
        edges.emplace_back(base_index + 3, base_index + 7); // v3 -> v7
    }
    
    // 将顶点和边写入 PLY 文件
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vertices.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "element edge " << edges.size() << "\n";
    out << "property int vertex1\n";
    out << "property int vertex2\n";
    out << "end_header\n";
    out.flush(); // 确保头部内容写入文件
    // 写入顶点
    for (int i = 0; i < vertices.size(); ++i)
    {
        out << (float)vertices[i].x() << " "
            << (float)vertices[i].y() << " "
            << (float)vertices[i].z() << " "
            << (int)colors[i].r() << " "
            << (int)colors[i].g() << " "
            << (int)colors[i].b() << "\n";
    }
    // 写入边
    for (const auto& edge : edges)
    {
        out  << std::get<0>(edge) << " " << std::get<1>(edge) << "\n";
    }
    out.close();
}

void PoissonReconstruction::outputBboxF0()
{
    std::string filename = "../output/checkbbox/F0Bbox.ply";
    std::ofstream out(filename);
    if (!out)
    {
        p_logger->error("Error: cannot open file {}", filename);
        return;
    }
    std::vector<Point> vertices; // 用于存储所有顶点
    std::vector<std::tuple<std::size_t, std::size_t>> edges; // 用于存储所有边（索引对）
    std::vector<CGAL::Color> colors;
    int cnt_D = 0;
    for(auto node : p_octree->traverse<LevelTraversal>(p_octree->depth()))
        cnt_D++;
    if(cnt_D != m_F02Node.size())
        p_logger->info("cnt_D++ != m_F02Node.size(), cnt_D {}, m_F02Node {}", cnt_D, m_F02Node.size() );
    for(auto f0 : m_F02Node)
    {
        NodeIndex node = f0.first;
        // 为当前节点的 8 个顶点添加颜色
        CGAL::Color node_color = CGAL::Color(255, 0, 0);
        for (int i = 0; i < 8; ++i)
            colors.push_back(node_color);

        // 获取节点的包围盒
        OctreeBbox3 bbox = p_octree->bbox(node);

        // 计算包围盒的 8 个顶点
        Point v0(bbox.xmin(), bbox.ymin(), bbox.zmin());
        Point v1(bbox.xmax(), bbox.ymin(), bbox.zmin());
        Point v2(bbox.xmax(), bbox.ymax(), bbox.zmin());
        Point v3(bbox.xmin(), bbox.ymax(), bbox.zmin());
        Point v4(bbox.xmin(), bbox.ymin(), bbox.zmax());
        Point v5(bbox.xmax(), bbox.ymin(), bbox.zmax());
        Point v6(bbox.xmax(), bbox.ymax(), bbox.zmax());
        Point v7(bbox.xmin(), bbox.ymax(), bbox.zmax());

        // 获取当前顶点的起始索引
        std::size_t base_index = vertices.size();
        // 添加顶点到顶点列表
        vertices.push_back(v0);
        vertices.push_back(v1);
        vertices.push_back(v2);
        vertices.push_back(v3);
        vertices.push_back(v4);
        vertices.push_back(v5);
        vertices.push_back(v6);
        vertices.push_back(v7);
        // 添加边到边列表（每条边由两个顶点索引组成）
        edges.emplace_back(base_index + 0, base_index + 1); // v0 -> v1
        edges.emplace_back(base_index + 1, base_index + 2); // v1 -> v2
        edges.emplace_back(base_index + 2, base_index + 3); // v2 -> v3
        edges.emplace_back(base_index + 3, base_index + 0); // v3 -> v0
        edges.emplace_back(base_index + 4, base_index + 5); // v4 -> v5
        edges.emplace_back(base_index + 5, base_index + 6); // v5 -> v6
        edges.emplace_back(base_index + 6, base_index + 7); // v6 -> v7
        edges.emplace_back(base_index + 7, base_index + 4); // v7 -> v4
        edges.emplace_back(base_index + 0, base_index + 4); // v0 -> v4
        edges.emplace_back(base_index + 1, base_index + 5); // v1 -> v5
        edges.emplace_back(base_index + 2, base_index + 6); // v2 -> v6
        edges.emplace_back(base_index + 3, base_index + 7); // v3 -> v7
    }
    
    // 将顶点和边写入 PLY 文件
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vertices.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "element edge " << edges.size() << "\n";
    out << "property int vertex1\n";
    out << "property int vertex2\n";
    out << "end_header\n";
    out.flush(); // 确保头部内容写入文件
    // 写入顶点
    for (int i = 0; i < vertices.size(); ++i)
    {
        out << (float)vertices[i].x() << " "
            << (float)vertices[i].y() << " "
            << (float)vertices[i].z() << " "
            << (int)colors[i].r() << " "
            << (int)colors[i].g() << " "
            << (int)colors[i].b() << "\n";
    }
    // 写入边
    for (const auto& edge : edges)
    {
        out  << std::get<0>(edge) << " " << std::get<1>(edge) << "\n";
    }
    out.close();
}

void PoissonReconstruction::outputTriInter(Point& point, std::vector<std::pair<NodeIndex,Point>>& neighborCenter)
{
    static int cnt = 0;
    std::string filePath = "../output/CheckTriInter";
    boost::filesystem::create_directory(filePath);
    std::string filename = filePath + "/Neighbor_" + std::to_string(cnt) + ".ply";
    std::ofstream stream(filename);
    cnt++;
    if(!stream) {
        p_logger->error("Error: cannot open file {}", filename);
        return;
    }

    std::vector<Point> vertices; // 用于存储所有顶点
    std::vector<std::tuple<std::size_t, std::size_t>> edges; // 用于存储所有边（索引对）
    std::vector<CGAL::Color> colors;

    for(int i=0;i<neighborCenter.size();++i)
    {
        NodeIndex node = neighborCenter[i].first;
        // 为当前节点的 8 个顶点添加颜色
        CGAL::Color node_color = CGAL::Color(i / 8.0 * 255.0, 0, 0);
        for (int j = 0; j < 8; ++j)
            colors.push_back(node_color);

        // 获取节点的包围盒
        OctreeBbox3 bbox = p_octree->bbox(node);

        // 计算包围盒的 8 个顶点
        Point v0(bbox.xmin(), bbox.ymin(), bbox.zmin());
        Point v1(bbox.xmax(), bbox.ymin(), bbox.zmin());
        Point v2(bbox.xmax(), bbox.ymax(), bbox.zmin());
        Point v3(bbox.xmin(), bbox.ymax(), bbox.zmin());
        Point v4(bbox.xmin(), bbox.ymin(), bbox.zmax());
        Point v5(bbox.xmax(), bbox.ymin(), bbox.zmax());
        Point v6(bbox.xmax(), bbox.ymax(), bbox.zmax());
        Point v7(bbox.xmin(), bbox.ymax(), bbox.zmax());

        // 获取当前顶点的起始索引
        std::size_t base_index = vertices.size();
        // 添加顶点到顶点列表
        vertices.push_back(v0);
        vertices.push_back(v1);
        vertices.push_back(v2);
        vertices.push_back(v3);
        vertices.push_back(v4);
        vertices.push_back(v5);
        vertices.push_back(v6);
        vertices.push_back(v7);
        // 添加边到边列表（每条边由两个顶点索引组成）
        edges.emplace_back(base_index + 0, base_index + 1); // v0 -> v1
        edges.emplace_back(base_index + 1, base_index + 2); // v1 -> v2
        edges.emplace_back(base_index + 2, base_index + 3); // v2 -> v3
        edges.emplace_back(base_index + 3, base_index + 0); // v3 -> v0
        edges.emplace_back(base_index + 4, base_index + 5); // v4 -> v5
        edges.emplace_back(base_index + 5, base_index + 6); // v5 -> v6
        edges.emplace_back(base_index + 6, base_index + 7); // v6 -> v7
        edges.emplace_back(base_index + 7, base_index + 4); // v7 -> v4
        edges.emplace_back(base_index + 0, base_index + 4); // v0 -> v4
        edges.emplace_back(base_index + 1, base_index + 5); // v1 -> v5
        edges.emplace_back(base_index + 2, base_index + 6); // v2 -> v6
        edges.emplace_back(base_index + 3, base_index + 7); // v3 -> v7
    }
    vertices.push_back(point);
    colors.push_back(CGAL::Color(255.0, 255.0, 0));

    // 将顶点和边写入 PLY 文件
    stream << "ply\n";
    stream << "format ascii 1.0\n";
    stream << "element vertex " << vertices.size() << "\n";
    stream << "property float x\n";
    stream << "property float y\n";
    stream << "property float z\n";
    stream << "property uchar red\n";
    stream << "property uchar green\n";
    stream << "property uchar blue\n";
    stream << "element edge " << edges.size() << "\n";
    stream << "property int vertex1\n";
    stream << "property int vertex2\n";
    stream << "end_header\n";
    stream.flush(); // 确保头部内容写入文件
    // 写入顶点
    for (int i = 0; i < vertices.size(); ++i)
    {
        stream << (float)vertices[i].x() << " "
            << (float)vertices[i].y() << " "
            << (float)vertices[i].z() << " "
            << (int)colors[i].r() << " "
            << (int)colors[i].g() << " "
            << (int)colors[i].b() << "\n";
    }
    // 写入边
    for (const auto& edge : edges)
    {
        stream  << std::get<0>(edge) << " " << std::get<1>(edge) << "\n";
    }
    stream.close();
}

void PoissonReconstruction::outputMarchingCubesResuilt() 
{
    std::string filePath("../output/mc_polygon");
    boost::filesystem::create_directory(filePath);
    std::string filename = filePath + "/mc.ply";

    /* reverse triangles */
    for(int i=0;i<m_polygon_soup.size();++i) {
        std::vector<int>& triangle = m_polygon_soup[i];
        for(int j=0;j<triangle.size()/2;j++) {
            std::swap(triangle[j], triangle[triangle.size()-1-j]);
        }
    }

    if(CGAL::IO::write_polygon_soup( filename, m_polygon_points, m_polygon_soup)) {
        p_logger->info("Marching Cubes result output done!");
    }
    else {
        p_logger->error("Error in outputting marching cubes resuilt");
    }
}
/// @}

/// @brief protected:
/// @name Protected Static Member Variables
/// @{
/// @}
/// @name Protected Member Variables
/// @{
/// @}
/// @name Protected Operatiors
/// @{
/// @}
/// @name Protected Operations
/// @{
/// @}
/// @name Protected Access
/// @{
/// @}
/// @name Protected Inquiry
/// @{
/// @}

/// @brief private:
/// @name Private Static Member Variables
/// @{
/// @}
/// @name Private Member Variables
/// @{
/// @}
/// @name Private Operatiors
/// @{
/// @}
/// @name Private Operations
/// @{
std::pair<NodeIndex, Point> PoissonReconstruction::findAdjacentNodeCenter(NodeIndex const& node, int dir1, int dir2, int dir3, int depth)
{
    if(dir2 == dir3)
    {
        if(dir1 == dir2)
        {
            NodeIndex adjNode1 = splitAndGetNode(node, dir1, depth);
            return std::pair(adjNode1,calBboxCen(adjNode1));
        }
        else
        {
            NodeIndex adjNode1 = splitAndGetNode(node, dir1, depth);
            NodeIndex adjNode2 = splitAndGetNode(adjNode1, dir2, depth);
            return std::pair(adjNode2,calBboxCen(adjNode2));
        }
    }
    else
    {
        NodeIndex adjNode1 = splitAndGetNode(node, dir1, depth);
        NodeIndex adjNode2 = splitAndGetNode(adjNode1, dir2, depth);
        NodeIndex adjNode3 = splitAndGetNode(adjNode2, dir3, depth);
        return std::pair(adjNode3,calBboxCen(adjNode3));
    }
}

NodeIndex PoissonReconstruction::splitAndGetNode(NodeIndex const& node, int dir, int depth)
{
    auto adjNodeOpt = p_octree->adjacent_node(node, dir);
    NodeIndex adjNode = -1;
    if(adjNodeOpt.has_value())
        adjNode = *adjNodeOpt;
    else
    {
        p_logger->error("Adjacent node error! out");
        exit(0);
    }
    int adjNodeOptDepth = p_octree->depth(adjNode);
    // p_logger->info("Adjacent node : {}, Adjacent node depth : {}, node depth : {}", adjNode, adjNodeOptDepth, depth);
    while(adjNodeOptDepth < depth)
    {
        p_octree->split(adjNode);
        adjNodeOpt = p_octree->adjacent_node(node, dir);
        if(adjNodeOpt.has_value())
            adjNode = *adjNodeOpt;
        else
        {
            p_logger->error("Adjacent node error! inner");
            exit(0);
        }
        adjNodeOptDepth = p_octree->depth(adjNode);
        // p_logger->info("Splitted Adjacent node : {}, Depth : {}", adjNode, adjNodeOptDepth );
    }
    return adjNode;
}

Point PoissonReconstruction::calBboxCen(NodeIndex const& node)
{
    OctreeBbox3 bbox = p_octree->bbox(node);
    double x_cen = 0.5 * (bbox.xmax() + bbox.xmin());
    double y_cen = 0.5 * (bbox.ymax() + bbox.ymin());
    double z_cen = 0.5 * (bbox.zmax() + bbox.zmin());
    return Point(x_cen, y_cen, z_cen);
}

void PoissonReconstruction::trilinearInterpolation(std::vector<std::pair<NodeIndex,Point>> const& bboxcenter, Point const& sp, Vector const& sn)
{
    /**
     * @details The store sequence of node center. y axis is upward. Local coordinate origin is 7
     *   y                  3------2
     *    |                /|     /|
     *    |               / |    / |
     *    7----x         1------0  |
     *   /               |  7---|--6
     *  /                | /    | /
     * z                 |/     |/
     *                   5------4    
     */
    double spx = sp.x(), spy = sp.y(), spz = sp.z();
    double xmax = bboxcenter[0].second.x(), xmin = bboxcenter[1].second.x(),
           ymax = bboxcenter[0].second.y(), ymin = bboxcenter[4].second.y(),
           zmax = bboxcenter[0].second.z(), zmin = bboxcenter[2].second.z();
    double ex = xmax - xmin, ey = ymax - ymin, ez = zmax - zmin;
    double rx = (spx - xmin) / ex, ry = (spy - ymin) / ey, rz = (spz - zmin) / ez;
    double alpha0 = rx * ry * rz, 
           alpha1 = (1 - rx) * ry * rz, 
           alpha2 = rx * ry * (1 - rz), 
           alpha3 = (1 - rx) * ry * (1 - rz),
           alpha4 = rx * (1 - ry) * rz, 
           alpha5 = (1 - rx) * (1 - ry) * rz, 
           alpha6 = rx * (1 - ry) * (1 - rz), 
           alpha7 = (1 - rx) * (1 - ry) * (1 - rz); 
    std::vector<double> alphas({alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7});
    Eigen::Vector3d normal(sn.x(), sn.y(), sn.z());
    normal.normalize();
    for(int i=0;i<8;++i)
    {
        if(m_alphaOS.find(bboxcenter[i].first) == m_alphaOS.end())
            m_alphaOS[bboxcenter[i].first] = alphas[i] * normal;
        else
            m_alphaOS[bboxcenter[i].first] += alphas[i] * normal;
    }
}

void PoissonReconstruction::calOverlapRecursive()
{   
    int intertime = 0;
    int cnt = 0;
    for(auto node : p_octree->traverse<PreorderTraversal>()) {  
        Bbox_3 nodeBbox = p_octree->bbox(node).bbox();
        nodeBbox.scale(3.0);
        std::vector<NodeIndex> overlapLeavesNodes;
        OverlapLeavesNodes(node, std::back_inserter(overlapLeavesNodes));
        for(auto interNode : overlapLeavesNodes) {
            NodeIndex node2 = interNode;
            int whilecnt = 0;
            while(1) {
                if(m_overlap_set[node].find(node2) == m_overlap_set[node].end()) {
                    Bbox_3 node2Bbox = p_octree->bbox(node2).bbox();
                    node2Bbox.scale(3.0);
                    const auto result = CGAL::intersection(nodeBbox,node2Bbox);
                    if(result) {
                        if(const Bbox_3* interbbox = std::get_if<Bbox_3>(&*result)) {
                            double xb = (*interbbox).xmin(), xe = (*interbbox).xmax(), 
                                   yb = (*interbbox).ymin(), ye = (*interbbox).ymax(), 
                                   zb = (*interbbox).zmin(), ze = (*interbbox).zmax(); 
                            if(std::fabs(xe-xb) > 1e-10 && std::fabs(ye-yb) > 1e-10 && std::fabs(ze-zb) > 1e-10) 
                                m_overlap_map_map[node][node2] = *interbbox;
                        }
                    }
                    // else {
                    //     p_logger->error("Node {} and Node {} not intersect!", node, node2);
                    //     p_logger->error("Node {} depth {}  Node{} depth {}", node, p_octree->depth(node),node2, p_octree->depth(node2));
                    //     output3Bbox(node, node2);
                    //     p_logger->error("while times : {}", whilecnt);
                    //     // p_logger->error("Node {} Bbox : {} {} {} {} {} {}",
                    //     //                  node, nodeBbox.xmin(), nodeBbox.xmax(), nodeBbox.ymin(), 
                    //     //                        nodeBbox.ymax(), nodeBbox.zmin(), nodeBbox.zmax());
                    //     // p_logger->error("Node {} Bbox : {} {} {} {} {} {}",
                    //     //                  node2, node2Bbox.xmin(), node2Bbox.xmax(), node2Bbox.ymin(),
                    //     //                         node2Bbox.ymax(), node2Bbox.zmin(), node2Bbox.zmax());
                    // }
                    m_overlap_set[node].insert(node2);
                }
                if(p_octree->is_root(node2))
                    break;
                else
                    node2 = p_octree->parent(node2);
                whilecnt++;
            }
        }
        intertime += m_overlap_map_map[node].size();
        cnt++;
    }
    p_logger->info("Intersect Times : {}, Nodes : {}", intertime, cnt);

    int cnt2 = 0;
    for(auto node : m_overlap_map_map) {
        NodeIndex nodeID = node.first;
        for(auto node2 : node.second) {
            NodeIndex node2ID = node2.first;
            if(m_overlap_map_map[node2ID].find(nodeID) == m_overlap_map_map[node2ID].end()) {
                m_overlap_map_map[node2ID][nodeID] = m_overlap_map_map[nodeID][node2ID];
            }
        }
    }

    cnt2 = 0;
    for(auto node : m_overlap_map_map) {
        NodeIndex nodeID = node.first;
        for(auto node2 : node.second) {
            NodeIndex node2ID = node2.first;
            if(m_overlap_map_map[node2ID].find(nodeID) == m_overlap_map_map[node2ID].end()) {
                if(cnt2<30) {
                    p_logger->info("Node {} Node {} not symmetric map map.", nodeID, node2ID);
                    cnt2++;
                }
            }
        }
    }

    // cnt2 = 0;
    // for(auto node : m_overlap_set) {
    //     NodeIndex nodeID = node.first;
    //     for(auto node2 : node.second) {
    //         if(m_overlap_set[node2].find(nodeID) == m_overlap_set[node2].end()) {
    //             if(cnt2==0) {
    //                 p_logger->info("Node {} Node {} not symmetric set.", nodeID, node2);
    //                 cnt2++;
    //             }
    //         }
    //     }
    // }

    p_logger->info("Overlap Symmetric Check Done.");
}

void PoissonReconstruction::calOverlap()
{
    m_overlap = std::unordered_map<NodeIndex, 
                                   std::vector<std::pair<NodeIndex,
                                                         Bbox_3>>>();
    int nodecnt = 0;
    int cnt=0;
    for(auto node1 : p_octree->traverse<PreorderTraversal>())
    {
        OctreeBbox3 obbox31 = p_octree->bbox(node1);
        Bbox_3 bbox31 = obbox31.bbox();
        bbox31.scale(1.5);
        for(auto node2 : p_octree->traverse<PreorderTraversal>())
        {
            OctreeBbox3 obbox32 = p_octree->bbox(node2);
            Bbox_3 bbox32 = obbox32.bbox();
            bbox32.scale(1.5);
            const auto result = CGAL::intersection(bbox31, bbox32);
            if(result)
            {
                if(const Bbox_3* interbbox = std::get_if<Bbox_3>(&*result))
                {
                    if(m_overlap.find(node1) == m_overlap.end())
                        m_overlap[node1] = std::vector<std::pair<NodeIndex,Bbox_3>>();
                    m_overlap[node1].push_back({node2,*interbbox});
                    cnt++;
                }
            }
        }

        nodecnt++;
        if(nodecnt % 1000000){
            double percent = nodecnt / m_num_of_nodes;
            p_logger->info("{} %", percent * 100);
        }
    }


    // for(int i=0;i<m_num_of_nodes;++i)
    //     m_overlap_vec.push_back(std::unordered_map<NodeIndex,Bbox_3>());
    // p_logger->info("begin to cal overlap");
    // int cnt = 0;
    // int nodecnt = 0;
    // for(auto node1 : p_octree->traverse<PreorderTraversal>())
    // {
    //     nodecnt++;
    //     if(nodecnt % 10000){
    //         double percent = nodecnt / m_num_of_nodes;
    //         p_logger->info("{} %", percent * 100);
    //     }
    //     OctreeBbox3 obbox31 = p_octree->bbox(node1);
    //     Bbox_3 bbox31 = obbox31.bbox();
    //     bbox31.scale(1.5);
    //     for(auto node2 : p_octree->traverse<PreorderTraversal>())
    //     {
    //         OctreeBbox3 obbox32 = p_octree->bbox(node2);
    //         Bbox_3 bbox32 = obbox32.bbox();
    //         bbox32.scale(1.5);
    //         const auto result = CGAL::intersection(bbox31, bbox32);
    //         if(result)
    //         {
    //             if(const Bbox_3* interbbox = std::get_if<Bbox_3>(&*result))
    //             {
    //                 m_overlap_vec[node1][node2] = *interbbox;
    //                 cnt++;
    //             }
    //         }
    //     }
    // }
    // /**
    //  * check overlap 
    //  */
    // for(int i=0;i<m_overlap_vec.size();++i)
    // {
    //     NodeIndex node1ID = i;
    //     for(auto node2 : m_overlap_vec[i])
    //     {
    //         Bbox_3 overlap12 = node2.second;
    //         NodeIndex node2ID = node2.first;
    //         if(m_overlap_vec[node2ID].find(node1ID) == m_overlap_vec[node2ID].end())
    //             p_logger->error("Node {} intersec Node {}, but reverse don't", node1ID, node2ID);
    //         else
    //         {
    //             Bbox_3 overlap21 = m_overlap_vec[node2ID][node1ID];
    //             if(overlap12 != overlap21)
    //                 p_logger->error("Node {} intersec Node {}, but bbox not same", node1ID, node2ID);
    //         }
    //     }
    // }
    // p_logger->info("Overlap is symmetric.");
    // int cnt = 0;
    // for(auto node : p_octree->traverse<PreorderTraversal>())
    // {
    //     auto data = p_octree->data(node);
    //     // if(p_octree->data(node).size() == 0)
    //     //     continue;
    //     // Point& point = m_points_set.point(p_octree->data(node)[0]);
    //     // if(m_virtual_points.find(point) != m_virtual_points.end())
    //     //     continue;
    //     OctreeBbox3 obbox3 = p_octree->bbox(node);
    //     Bbox_3 bbox3 = obbox3.bbox();
    //     bbox3.scale(1.5);
    //     for(auto node2 : p_octree->traverse<PreorderTraversal>())
    //     {
    //         auto data2 = p_octree->data(node2);
    //         // if(p_octree->data(node2).size() == 0)
    //         //     continue;
    //         // Point& point2 = m_points_set.point(p_octree->data(node2)[0]);
    //         // if(m_virtual_points.find(point2) != m_virtual_points.end())
    //         //     continue;
    //         OctreeBbox3 obbox32 = p_octree->bbox(node2);
    //         Bbox_3 bbox32 = obbox32.bbox();
    //         bbox32.scale(1.5);
    //         const auto result = CGAL::intersection(bbox3, bbox32);
    //         if(result)
    //         {
    //             if(const Bbox_3* interbbox = std::get_if<Bbox_3>(&*result))
    //             {
    //                 if(m_overlap.find(node) == m_overlap.end())
    //                     m_overlap[node] = (std::vector<std::pair<NodeIndex,
    //                                                              Bbox_3>>());
    //                 m_overlap[node].push_back(std::make_pair(node2, *interbbox));
    //                 cnt++;
    //             }
    //         }
    //     }
    // }
    p_logger->info("Intersection times : {}", cnt);
}

void PoissonReconstruction::calOverlapD()
{
    m_overlap = std::unordered_map<NodeIndex, 
                                    std::vector<std::pair<NodeIndex,
                                                     Bbox_3>>>();
    int cnt = 0;
    int dep = p_octree->depth();
    for(auto node1 : p_octree->traverse<LevelTraversal>(dep))
    {
        if(m_overlap.find(node1) == m_overlap.end())
            m_overlap[node1] = std::vector<std::pair<NodeIndex, Bbox_3>>();
        OctreeBbox3 obbox31 = p_octree->bbox(node1);
        Bbox_3 bbox31 = obbox31.bbox();
        bbox31.scale(3);
        for(auto node2 : p_octree->traverse<LevelTraversal>(dep))
        {
            OctreeBbox3 obbox32 = p_octree->bbox(node2);
            Bbox_3 bbox32 = obbox32.bbox();
            bbox32.scale(3);
            const auto result = CGAL::intersection(bbox31, bbox32);
            if(result)
            {
                if(const Bbox_3* interbbox = std::get_if<Bbox_3>(&*result))
                {
                    m_overlap[node1].push_back({node2, *interbbox});
                    cnt++;
                }
            }
        }
    }
    p_logger->info("Intersection times : {}", cnt);
}

/**
 * @param t0 the global loc
 * @param dir 
 * |axis | dir | 
 * |  x  |  0  |
 * |  y  |  1  |
 * |  z  |  2  |
 */
double PoissonReconstruction::calBboxFilterFunctionConvolve3(double to, NodeIndex node, int dir)
{
    OctreeBbox3 obbox = p_octree->bbox(node);
    Bbox_3 bbox = obbox.bbox();
    bbox.scale(1.5);
    double tb = 0.0, te = 0.0;
    switch(dir){
        case 0:
            tb = bbox.xmin(), te = bbox.xmax();
            break;
        case 1:
            tb = bbox.ymin(), te = bbox.ymax();
            break;
        case 2:
            tb = bbox.zmin(), te = bbox.zmax();
            break;
        default:
            p_logger->error("error!");
            break;
    }
    double t = (to - tb) / (te - tb) * 3.0 - 1.5;
    double ow = te - tb;
    // p_logger->info("noDiffer : {}", t);
    return calBboxFilterFunctionConvolve3(t) / ow;
}

double PoissonReconstruction::calBboxFilterFunctionConvolve3Differ(double to, NodeIndex node, int dir)
{
    OctreeBbox3 obbox = p_octree->bbox(node);
    Bbox_3 bbox = obbox.bbox();
    bbox.scale(1.5);
    double tb = 0.0, te = 0.0;
    switch(dir){
        case 0:
            tb = bbox.xmin(), te = bbox.xmax();
            break;
        case 1:
            tb = bbox.ymin(), te = bbox.ymax();
            break;
        case 2:
            tb = bbox.zmin(), te = bbox.zmax();
            break;
        default:
            p_logger->error("error!");
            break;
    }
    double t = (to - tb) / (te - tb) * 3.0 - 1.5;
    // p_logger->info("differ : {} ", t);
    return calBboxFilterFunctionConvolve3Differ(t);
}

double PoissonReconstruction::calBboxFilterFunctionConvolve3Differ2(double to, NodeIndex node, int dir)
{
    OctreeBbox3 obbox = p_octree->bbox(node);
    Bbox_3 bbox = obbox.bbox();
    bbox.scale(1.5);
    double tb = 0.0, te = 0.0;
    switch(dir){
        case 0:
            tb = bbox.xmin(), te = bbox.xmax();
            break;
        case 1:
            tb = bbox.ymin(), te = bbox.ymax();
            break;
        case 2:
            tb = bbox.zmin(), te = bbox.zmax();
            break;
        default:
            p_logger->error("error!");
            break;
    }
    double t = (to - tb) / (te - tb) * 3.0 - 1.5;
    // p_logger->info("Differ2 : {}", t);
    return calBboxFilterFunctionConvolve3Differ2(t);
}

double PoissonReconstruction::calv0(NodeIndex node1, NodeIndex node2, Bbox_3& overlap)
{
    // double xb = overlap.xmin(), xe = overlap.xmax(),
    //        yb = overlap.ymin(), ye = overlap.ymax(),
    //        zb = overlap.zmin(), ze = overlap.zmax();
    // if( std::fabs(xe-xb) < 1e-5 || std::fabs(ye-yb) < 1e-5 || std::fabs(ze-zb) < 1e-5)
    //     p_logger->info("illoverlap");
    // auto fg_x = [node1, node2, this](double t0)->double {
    //     double cbffc = this->calBboxFilterFunctionConvolve3(t0, node2, 0);
    //     double cbffcd = this->calBboxFilterFunctionConvolve3Differ(t0, node1, 0);
    //     // this->p_logger->info("cbffc : {} , cbffcd : {}", cbffc, cbffcd);
    //     return cbffc * cbffcd;
    // };
    // auto fg_y = [node1, node2, this](double t0)->double {
    //     double cbffc = this->calBboxFilterFunctionConvolve3(t0, node2, 1);
    //     double cbffcd = this->calBboxFilterFunctionConvolve3Differ(t0, node1, 1);
    //     // this->p_logger->info("cbffc : {} , cbffcd : {}", cbffc, cbffcd);
    //     return cbffc * cbffcd;
    // };
    // auto fg_z = [node1, node2, this](double t0)->double {
    //     double cbffc = this->calBboxFilterFunctionConvolve3Differ(t0, node1, 2);
    //     double cbffcd = this->calBboxFilterFunctionConvolve3(t0, node2, 2);
    //     // this->p_logger->info("cbffc : {} , cbffcd : {}", cbffc, cbffcd);
    //     return cbffc * cbffcd;  
    // };
    // double x = MyUtility::Gauss_Legendre(xb, xe, fg_x) * m_alphaOS[node1].x();
    // double y = MyUtility::Gauss_Legendre(yb, ye, fg_y) * m_alphaOS[node1].y();
    // double z = MyUtility::Gauss_Legendre(zb, ze, fg_z) * m_alphaOS[node1].z();
    // // p_logger->info("x : {}, y : {}, z : {} ", x, y, z);
    // return x * y * z;


    double xb = overlap.xmin(), xe = overlap.xmax(),
           yb = overlap.ymin(), ye = overlap.ymax(),
           zb = overlap.zmin(), ze = overlap.zmax();
    if( std::fabs(xe-xb) < 1e-10 || std::fabs(ye-yb) < 1e-10 || std::fabs(ze-zb) < 1e-10)
        p_logger->info("illoverlap");
    
    auto fg = [this, node1, node2](double x, double y, double z)
    {
        Point q(x,y,z);
        Eigen::Vector3d alphaOS = this->m_alphaOS[node1];
        double f = alphaOS.x()  * this->calF0PX(q, node1);
               f += alphaOS.y() * this->calF0PY(q, node1);
               f += alphaOS.z() * this->calF0PZ(q, node1);
        double g = this->calF0(q, node2);
        return f * g;
    };
    double ans = MyUtility::Gauss_Legendre3(xb, yb, zb, xe, ye, ze, fg);
    return ans;
}

double PoissonReconstruction::call0(NodeIndex node1, NodeIndex node2, Bbox_3& overlap)
{
    // double xb = overlap.xmin(), xe = overlap.xmax(),
    //        yb = overlap.ymin(), ye = overlap.ymax(),
    //        zb = overlap.zmin(), ze = overlap.zmax();
    // if( std::fabs(xe-xb) < 1e-5 || std::fabs(ye-yb) < 1e-5 || std::fabs(ze-zb) < 1e-5)
    //     p_logger->info("illoverlap"); 
    // auto fg_x = [node1, node2, this](double t0)->double {
    //     double cbffcd = this->calBboxFilterFunctionConvolve3Differ2(t0, node1, 0);
    //     double cbffc = this->calBboxFilterFunctionConvolve3(t0, node2, 0);
    //     // this->p_logger->info("cbffc : {} , cbffcd : {}", cbffc, cbffcd);
    //     return cbffc * cbffcd;
    // };
    // auto fg_y = [node1, node2, this](double t0)->double {
    //     double cbffcd = this->calBboxFilterFunctionConvolve3Differ2(t0, node1, 1);
    //     double cbffc = this->calBboxFilterFunctionConvolve3(t0, node2, 1);
    //     // this->p_logger->info("cbffc : {} , cbffcd : {}", cbffc, cbffcd);
    //     return cbffc * cbffcd;
    // };
    // auto fg_z = [node1, node2, this](double t0)->double {
    //     double cbffcd = this->calBboxFilterFunctionConvolve3Differ2(t0, node1, 2);
    //     double cbffc = this->calBboxFilterFunctionConvolve3(t0, node2, 2);
    //     // this->p_logger->info("cbffc : {} , cbffcd : {}", cbffc, cbffcd);
    //     return cbffc * cbffcd;  
    // };
    // auto fg_x2 = [node1, node2, this](double t0)->double {
    //     double cbffcd = this->calBboxFilterFunctionConvolve3Differ2(t0, node2, 0);
    //     double cbffc = this->calBboxFilterFunctionConvolve3(t0, node1, 0);
    //     // this->p_logger->info("cbffc 2: {} , cbffcd : {}", cbffc, cbffcd);
    //     return cbffc * cbffcd;
    // };
    // auto fg_y2 = [node1, node2, this](double t0)->double {
    //     double cbffcd = this->calBboxFilterFunctionConvolve3Differ2(t0, node2, 1);
    //     double cbffc = this->calBboxFilterFunctionConvolve3(t0, node1, 1);
    //     // this->p_logger->info("cbffc 2: {} , cbffcd : {}", cbffc, cbffcd);
    //     return cbffc * cbffcd;
    // };
    // auto fg_z2 = [node1, node2, this](double t0)->double {
    //     double cbffcd = this->calBboxFilterFunctionConvolve3Differ2(t0, node2, 2);
    //     double cbffc = this->calBboxFilterFunctionConvolve3(t0, node1, 2);
    //     // this->p_logger->info("cbffc : {} , cbffcd : {}", cbffc, cbffcd);
    //     return cbffc * cbffcd;  
    // };
    // double x = MyUtility::Gauss_Legendre(xb, xe, fg_x);
    // double y = MyUtility::Gauss_Legendre(yb, ye, fg_y);
    // double z = MyUtility::Gauss_Legendre(zb, ze, fg_z);
    // double x_ = MyUtility::Gauss_Legendre(xb, xe, fg_x2);
    // double y_ = MyUtility::Gauss_Legendre(xb, xe, fg_y2);
    // double z_ = MyUtility::Gauss_Legendre(xb, xe, fg_z2);
    // double ans1 = x * y * z;
    // double ans2 = x_ * y_ * z_;
    // if(ans1 != ans2)
    // {
    //     p_logger->info("node1 node2 l0i : {}  node2 node1 l0i : {}", ans1, ans2);
    //     p_logger->info("x : {} y : {} z : {}  ... x_ : {} y_ : {} z_ : {} ",
    //                     x,y,z,x_,y_,z_);
    // }
    // // p_logger->info("x : {}, y : {}, z : {} ", x, y, z);
    // return x * y * z;
    static int cnt = 0;
    double  xb = overlap.xmin(), xe = overlap.xmax(),
            yb = overlap.ymin(), ye = overlap.ymax(),
            zb = overlap.zmin(), ze = overlap.zmax();
    if( std::fabs(xe-xb) < 1e-5 || std::fabs(ye-yb) < 1e-5 || std::fabs(ze-zb) < 1e-5)
        p_logger->info("illoverlap"); 
    
    // auto fg = [this, node1, node2](double x, double y, double z)
    // {
    //     Point q(x,y,z);
    //     double f = this->calF0PX2(q, node1) * this->calF0(q, node2);
    //     f +=       this->calF0PY2(q, node1) * this->calF0(q, node2);  
    //     f +=       this->calF0PZ2(q, node1) * this->calF0(q, node2);
    //     return f;
    // };
    // auto fg_ = [this, node1, node2](double x, double y, double z)
    // {
    //     Point q(x,y,z);
    //     double f = this->calF0PX2(q, node2) * this->calF0(q, node1);
    //     f +=       this->calF0PY2(q, node2) * this->calF0(q, node1);  
    //     f +=       this->calF0PZ2(q, node2) * this->calF0(q, node1);
    //     return f;
    // };

    auto fg = [this, node1, node2](double x, double y, double z)
    {
        Point q(x,y,z);
        double f = -1.0 * this->calF0PX(q, node1) * this->calF0PX(q, node2);
        f +=       -1.0 * this->calF0PY(q, node1) * this->calF0PY(q, node2);  
        f +=       -1.0 * this->calF0PZ(q, node1) * this->calF0PZ(q, node2);
        return f;
    };
    // auto fg_ = [this, node1, node2](double x, double y, double z)
    // {
    //     Point q(x,y,z);
    //     double f = -1.0 * this->calF0PX(q, node2) * this->calF0PX(q, node1);
    //     f +=       -1.0 * this->calF0PY(q, node2) * this->calF0PY(q, node1);  
    //     f +=       -1.0 * this->calF0PZ(q, node2) * this->calF0PZ(q, node1);
    //     return f;
    // };


    // // if(p_octree->depth(node1) != p_octree->depth(node2))
    // //     p_logger->info("Not same depth");

    // double ans_ = MyUtility::Gauss_Legendre3(xb, yb, zb, xe, ye, ze, fg_);
    double ans = MyUtility::Gauss_Legendre3(xb, yb, zb, xe, ye, ze, fg);
    // if(std::fabs(ans_ - ans) > 1e-2)
    // {
    //     p_logger->info("Node {} and {}  : lo {} != {} ", node1, node2, ans, ans_);
    //     OctreeBbox3 obbox1 = p_octree->bbox(node1), obbox2 = p_octree->bbox(node2);
    //     auto bbox1 = obbox1.bbox(); 
    //     auto bbox2 = obbox2.bbox();
    // }

    // if(node1 == 14960 && node2 == 14958)
    // {
    //     p_logger->info("bbox : {} {} {} \n {} {} {}", xb, yb, zb, xe, ye, ze);
    //     double ans2_ = MyUtility::Gauss_Legendre3_debug(xb, yb, zb, xe, ye, ze, node2, node1, fg_);
    //     double ans2 = MyUtility::Gauss_Legendre3_debug(xb, yb, zb, xe, ye, ze, node1, node2, fg);
    //     double ans3_ = MyUtility::Gauss_Legendre3(xb, yb, zb, xe, ye, ze, fg_);
    //     double ans3 = MyUtility::Gauss_Legendre3(xb, yb, zb, xe, ye, ze, fg);
    //     p_logger->info("node1 {} node2 {} ans3 {} ans3_ {}", 14960, 14958, ans , ans3_);
    // }

    // if(node1 == 14958 && node2 == 14960)
    // {
    //     p_logger->info("bbox : {} {} {} \n {} {} {}", xb, yb, zb, xe, ye, ze);
    //     double ans2_ = MyUtility::Gauss_Legendre3_debug(xb, yb, zb, xe, ye, ze, node2, node1, fg_);
    //     double ans2 = MyUtility::Gauss_Legendre3_debug(xb, yb, zb, xe, ye, ze, node1, node2, fg);
    //     double ans3_ = MyUtility::Gauss_Legendre3(xb, yb, zb, xe, ye, ze, fg_);
    //     double ans3 = MyUtility::Gauss_Legendre3(xb, yb, zb, xe, ye, ze, fg);
    //     p_logger->info("node1 {} node2 {} ans3 {} ans3_ {}", 14960, 14958, ans , ans3_);
    // }

    // if(cnt == 0)
    // {
    //     double ans2_ = MyUtility::Gauss_Legendre3_debug(xb, yb, zb, xe, ye, ze, 1, fg_);
    //     double ans2 = MyUtility::Gauss_Legendre3_debug(xb, yb, zb, xe, ye, ze, 2, fg);
    //     cnt ++;
    // }
    return ans;
}

double PoissonReconstruction::calBboxFilterFunctionConvolve3(double t)
{
    if(std::fabs(t) <= 0.5)
        return 0.75 - std::pow(t,2);
    else if(std::fabs(t) > 0.5 && std::fabs(t) <= 1.5)
        return 0.5 * std::pow(1.5 - std::fabs(t),2);
    else
        return 0;
}

double PoissonReconstruction::calBboxFilterFunctionConvolve3Differ(double t)
{
    if(std::fabs(t) <= 0.5)
        return -2.0 * t;
    else if( t < -0.5 && t >= -1.5)
        return 1.5 + t;
    else if( t > 0.5 && t <= 1.5)
        return -1.5 + t;
    else
        return 0;
}

double PoissonReconstruction::calBboxFilterFunctionConvolve3Differ2(double t)
{
    if(std::fabs(t) <= 0.5)
        return -2.0;
    else if( t < -0.5 && t >= -1.5)
        return 1.0;
    else if( t > 0.5 && t <= 1.5)
        return 1.0;
    else
        return 0;
}

double PoissonReconstruction::calF0(Point const& q, NodeIndex oNode)
{
    OctreeBbox3 obbox = p_octree->bbox(oNode);
    double owx = obbox.xmax() - obbox.xmin(), owy = obbox.ymax() - obbox.ymin(), owz = obbox.zmax() - obbox.zmin();
    Point ocPoint(obbox.xmin() + 0.5 * owx, obbox.ymin() + 0.5 * owy, obbox.zmin() + 0.5 * owz);
    double ans = 1.0 / (owx * owy * owz) * calBboxFilterFunctionConvolve3((q.x()-ocPoint.x()) / owx)
                                         * calBboxFilterFunctionConvolve3((q.y()-ocPoint.y()) / owy)
                                         * calBboxFilterFunctionConvolve3((q.z()-ocPoint.z()) / owz);
    return ans;
}

double PoissonReconstruction::calF0PX(Point const& q, NodeIndex oNode)
{
    OctreeBbox3 obbox = p_octree->bbox(oNode);
    assert(q.x() > obbox.xmin() && q.x() < obbox.xmax()
        && q.y() > obbox.ymin() && q.y() < obbox.ymax()
        && q.z() > obbox.zmin() && q.z() < obbox.zmax());

    double owx = obbox.xmax() - obbox.xmin(), owy = obbox.ymax() - obbox.ymin(), owz = obbox.zmax() - obbox.zmin();
    Point ocPoint(obbox.xmin() + 0.5 * owx, obbox.ymin() + 0.5 * owy, obbox.zmin() + 0.5 * owz);
    double ans = 1.0 / (owx * owy * owz) * calBboxFilterFunctionConvolve3Differ((q.x()-ocPoint.x()) / owx) * (1.0 / owx)
                                         * calBboxFilterFunctionConvolve3((q.y()-ocPoint.y()) / owy)
                                         * calBboxFilterFunctionConvolve3((q.z()-ocPoint.z()) / owz);
    return ans;
}

double PoissonReconstruction::calF0PY(Point const& q, NodeIndex oNode)
{
    OctreeBbox3 obbox = p_octree->bbox(oNode);
    double owx = obbox.xmax() - obbox.xmin(), owy = obbox.ymax() - obbox.ymin(), owz = obbox.zmax() - obbox.zmin();
    Point ocPoint(obbox.xmin() + 0.5 * owx, obbox.ymin() + 0.5 * owy, obbox.zmin() + 0.5 * owz);
    double ans = 1.0 / (owx * owy * owz) * calBboxFilterFunctionConvolve3((q.x()-ocPoint.x()) / owx) 
                                         * calBboxFilterFunctionConvolve3Differ((q.y()-ocPoint.y()) / owy) * (1.0 / owy)
                                         * calBboxFilterFunctionConvolve3((q.z()-ocPoint.z()) / owz);
    return ans;
}

double PoissonReconstruction::calF0PZ(Point const& q, NodeIndex oNode)
{
    OctreeBbox3 obbox = p_octree->bbox(oNode);
    double owx = obbox.xmax() - obbox.xmin(), owy = obbox.ymax() - obbox.ymin(), owz = obbox.zmax() - obbox.zmin();
    Point ocPoint(obbox.xmin() + 0.5 * owx, obbox.ymin() + 0.5 * owy, obbox.zmin() + 0.5 * owz);
    double ans = 1.0 / (owx * owy * owz) * calBboxFilterFunctionConvolve3((q.x()-ocPoint.x()) / owx) 
                                         * calBboxFilterFunctionConvolve3((q.y()-ocPoint.y()) / owy) 
                                         * calBboxFilterFunctionConvolve3Differ((q.z()-ocPoint.z()) / owz) * (1.0 / owz);
    return ans;
}

double PoissonReconstruction::calF0PX2(Point const& q, NodeIndex oNode)
{
    OctreeBbox3 obbox = p_octree->bbox(oNode);
    double owx = obbox.xmax() - obbox.xmin(), owy = obbox.ymax() - obbox.ymin(), owz = obbox.zmax() - obbox.zmin();
    Point ocPoint(obbox.xmin() + 0.5 * owx, obbox.ymin() + 0.5 * owy, obbox.zmin() + 0.5 * owz);
    double ans = 1.0 / (owx * owy * owz) * calBboxFilterFunctionConvolve3Differ2((q.x()-ocPoint.x()) / owx) * (1.0 / owx) * (1.0 / owx)
                                         * calBboxFilterFunctionConvolve3((q.y()-ocPoint.y()) / owy)
                                         * calBboxFilterFunctionConvolve3((q.z()-ocPoint.z()) / owz);
    return ans;
}

double PoissonReconstruction::calF0PY2(Point const& q, NodeIndex oNode)
{
    OctreeBbox3 obbox = p_octree->bbox(oNode);
    double owx = obbox.xmax() - obbox.xmin(), owy = obbox.ymax() - obbox.ymin(), owz = obbox.zmax() - obbox.zmin();
    Point ocPoint(obbox.xmin() + 0.5 * owx, obbox.ymin() + 0.5 * owy, obbox.zmin() + 0.5 * owz);
    double ans = 1.0 / (owx * owy * owz) * calBboxFilterFunctionConvolve3((q.x()-ocPoint.x()) / owx) 
                                         * calBboxFilterFunctionConvolve3Differ2((q.y()-ocPoint.y()) / owy) * (1.0 / owy) * (1.0 / owy)
                                         * calBboxFilterFunctionConvolve3((q.z()-ocPoint.z()) / owz);
    return ans;
}

double PoissonReconstruction::calF0PZ2(Point const& q, NodeIndex oNode)
{
    OctreeBbox3 obbox = p_octree->bbox(oNode);
    double owx = obbox.xmax() - obbox.xmin(), owy = obbox.ymax() - obbox.ymin(), owz = obbox.zmax() - obbox.zmin();
    Point ocPoint(obbox.xmin() + 0.5 * owx, obbox.ymin() + 0.5 * owy, obbox.zmin() + 0.5 * owz);
    double ans = 1.0 / (owx * owy * owz) * calBboxFilterFunctionConvolve3((q.x()-ocPoint.x()) / owx) 
                                         * calBboxFilterFunctionConvolve3((q.y()-ocPoint.y()) / owy) 
                                         * calBboxFilterFunctionConvolve3Differ2((q.z()-ocPoint.z()) / owz) * (1.0 / owz) * (1.0 / owz);
    return ans;
}

double PoissonReconstruction::calIndicator(Point const& point)
{
    static int cnt =0;
    std::string filename= "../output/indicator/indicator_" + std::to_string(cnt++) + ".txt";
    std::ofstream stream(filename);
    double ans = 0.0;
    for(int i=0;i<m_X.rows();++i) {
        ans += m_X(i) * calF0(point, i);
        if(calF0(point, i) != 0)
            stream << "Indicator : " << m_X(i) << " " << calF0(point,i) << " " << ans << std::endl;
    }
    // for(auto f0 : m_F02Node)
    // {
    //     NodeIndex node = f0.first;
    //     int nodeid = f0.second;
    //     ans += m_X(nodeid) * calF0(point, node);
    //     // p_logger->info("m_X {}  F0 {}", m_X(nodeid), calF0(point, node));
    //     stream << "Indicator : " << m_X(nodeid) << " " << calF0(point,node) << " " << ans << std::endl;
    // }
    stream.close();
    // outputBboxF0();
    return ans;

    // double ans = 0.0;
    // for(int i=0;i<m_num_of_nodes;++i)
    // {
    //     OctreeBbox3 obbox = p_octree->bbox(i);
    //     Bbox_3 bbox = obbox.bbox();
    //     bbox.scale(1.5);
    //     if(point.x() > bbox.xmin() && point.x() < bbox.xmax() &&
    //        point.y() > bbox.ymin() && point.y() < bbox.ymax() &&
    //        point.z() > bbox.zmin() && point.z() < bbox.zmax())
    //     {
    //         double local_x = (point.x() - bbox.xmin()) / (bbox.xmax() - bbox.xmin()) * 3.0 - 1.5;
    //         double local_y = (point.y() - bbox.ymin()) / (bbox.ymax() - bbox.ymin()) * 3.0 - 1.5;
    //         double local_z = (point.z() - bbox.zmin()) / (bbox.zmax() - bbox.zmin()) * 3.0 - 1.5;
    //         double temp =   calBboxFilterFunctionConvolve3(local_x) + 
    //                         calBboxFilterFunctionConvolve3(local_y) +
    //                         calBboxFilterFunctionConvolve3(local_z);
    //         ans += m_X(i) * temp;
    //     }
    // }
    // return ans;
}

void PoissonReconstruction::calNodeVerIndicatorAndIndex(NodeIndex node, std::vector<Point> & nodeVertices) 
{
    m_node_indicators[node] = std::array<double,8>();
    m_indices[node] = std::bitset<8>(0);
    for(int i=0;i<8;++i) {
        m_node_indicators[node][i] = calIndicator(nodeVertices[i]);
        if(m_node_indicators[node][i] <= m_scale_coefficient)
            m_indices[node].set(i,1);
    }      
}

bool PoissonReconstruction::checkSplit(NodeIndex node, std::array<int, 16>& allEdgeIDs, std::vector<Point>& nodeVertices) 
{
    std::array<std::array<int,2>, 12>& edgeTable = m_edgeTable;
    for(int i=0;i<16;i++) {
        if(allEdgeIDs[i] == -1) 
            continue;
        Segment edge(nodeVertices[edgeTable[allEdgeIDs[i]][0]], nodeVertices[edgeTable[allEdgeIDs[i]][1]]);
        int deepest = 0;
        auto isIntersectedPoint = [this, &edge, &deepest](NodeIndex const& node) {
            OctreeBbox3 interobbox = p_octree->bbox(node);
            const auto result = CGAL::intersection(edge, interobbox);
            if(const Point* interPoint = std::get_if<Point>(&*result)) {
                return true;
            }else if(const Segment* interSegment = std::get_if<Segment>(&*result)) {
                if(p_octree->depth(node) > deepest)
                    deepest = p_octree->depth(node);
                return false;
            } else {
                if(p_octree->depth(node) > deepest)
                    deepest = p_octree->depth(node);
                return false;
            }
        };
        std::vector<NodeIndex> interNodes;
        p_octree->intersected_nodes(edge, std::back_inserter(interNodes));
        interNodes.erase(std::remove_if(interNodes.begin(), interNodes.end(), isIntersectedPoint), interNodes.end());
        std::unordered_set<Segment> segSet;
        for(int j=0;j<interNodes.size();++j) {
            if(p_octree->depth(interNodes[j]) < deepest)
                continue;
            const auto result = CGAL::intersection(edge, p_octree->bbox(interNodes[j]));
            const Segment* interSeg = std::get_if<Segment>(&*result);
            segSet.insert(*interSeg);
        }
        if(segSet.size() > 1)
            return true;
    }
    return false;
}

std::unordered_set<Segment> PoissonReconstruction::finestEdges(Segment& edge, NodeIndex node)
{
    int deepest = 0;
    auto isIntersectedPoint = [this, &edge, &deepest](NodeIndex const& node) {
        OctreeBbox3 interobbox = p_octree->bbox(node);
        const auto result = CGAL::intersection(edge, interobbox);
        if(const Point* interPoint = std::get_if<Point>(&*result)) {
            return true;
        }else if(const Segment* interSegment = std::get_if<Segment>(&*result)) {
            if(p_octree->depth(node) > deepest)
                deepest = p_octree->depth(node);
            return false;
        } else {
            if(p_octree->depth(node) > deepest)
                deepest = p_octree->depth(node);
            return false;
        }
    };
    std::vector<NodeIndex> interNodes;
    p_octree->intersected_nodes(edge, std::back_inserter(interNodes));
    interNodes.erase(std::remove_if(interNodes.begin(), interNodes.end(), isIntersectedPoint), interNodes.end());
    std::unordered_set<Segment> segSet;
    for(int i=0;i<interNodes.size();++i) {
        if(p_octree->depth(interNodes[i]) < deepest)
            continue;
        const auto result = CGAL::intersection(edge, p_octree->bbox(interNodes[i]));
        const Segment* interSeg = std::get_if<Segment>(&*result);
        segSet.insert(*interSeg);
    }
    if(segSet.size() > 1 || segSet.size() == 0) {
        p_logger->info("Marching Cubes ----- InterNodes : {}", interNodes.size());
        p_logger->info("Marching Cubes ----- SegSet : {}", segSet.size());
    }
    return segSet;
}

std::variant<Point,int> PoissonReconstruction::finestZeroCrossing(Segment& edge, NodeIndex node)
{
    int deepest = 0;
    auto isIntersectedPoint = [this, &edge, &deepest](NodeIndex const& node) {
        OctreeBbox3 interobbox = p_octree->bbox(node);
        const auto result = CGAL::intersection(edge, interobbox);
        if(const Point* interPoint = std::get_if<Point>(&*result)) {
            return true;
        }else if(const Segment* interSegment = std::get_if<Segment>(&*result)) {
            if(p_octree->depth(node) > deepest)
                deepest = p_octree->depth(node);
            return false;
        } else {
            if(p_octree->depth(node) > deepest)
                deepest = p_octree->depth(node);
            return false;
        }
    };
    std::vector<NodeIndex> interNodes;
    p_octree->intersected_nodes(edge, std::back_inserter(interNodes));
    interNodes.erase(std::remove_if(interNodes.begin(), interNodes.end(), isIntersectedPoint), interNodes.end());
    std::unordered_set<Segment> segSet;
    for(int i=0;i<interNodes.size();++i) {
        if(p_octree->depth(interNodes[i]) < deepest)
            continue;
        const auto result = CGAL::intersection(edge, p_octree->bbox(interNodes[i]));
        const Segment* interSeg = std::get_if<Segment>(&*result);
        segSet.insert(*interSeg);
    }
    std::vector<Point> interPoints;
    for(auto it = segSet.begin(); it != segSet.end(); ++it) {
        auto result = interPolatePoint(*it);
        if(result.has_value())
            interPoints.push_back(*result);
    }
    if(interPoints.size() != 1 )
        return (int)interPoints.size();
    else
        return interPoints[0];
}

std::optional<Point> PoissonReconstruction::interPolatePoint(Segment const& edge)
{
    Point p1 = edge.point(0), p2 = edge.point(1);
    double indicator1 = calIndicator(p1);
    double indicator2 = calIndicator(p2);
    if((indicator1 > m_scale_coefficient && indicator2 > m_scale_coefficient) ||
       (indicator1 < m_scale_coefficient && indicator2 < m_scale_coefficient))
        return std::nullopt;
    else
        return InterPolatePoint(p1, p2, indicator1, indicator2);
}

Point PoissonReconstruction::InterPolatePoint(Point const& p1, Point const& p2, double indicator1, double indicator2)
{
    Eigen::Vector3d v1(p1.x(), p1.y(), p1.z()), v2(p2.x(), p2.y(), p2.z());
    double t = (m_scale_coefficient - indicator1) / (indicator2 - indicator1);
    Eigen::Vector3d v0 = t * (v2 - v1) + v1;
    return Point(v0.x(), v0.y(), v0.z());
}

/// @}
/// @name Private Access
/// @{
/// @}
/// @name Private Inquiry
/// @{
/// @}
