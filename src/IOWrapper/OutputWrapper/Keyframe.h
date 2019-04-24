#pragma once

#undef Success
#include <Eigen/Core>
#include "util/NumType.h"

#include <sstream>
#include <fstream>

namespace dso
{
    class CalibHessian;
    class FrameHessian;
    class FrameShell;

    namespace IOWrap
    {
        template<int ppp>
            struct InputPointSparse
            {
                float u;
                float v;
                float idepth;
                float idepth_hessian;
                float relObsBaseline;
                int numGoodRes;
               // unsigned char color[ppp];
                unsigned char status;
            };
        struct MyVertex
        {
            float point[3];
            //unsigned char color[4];
        };

        class Keyframe
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                Keyframe();
                ~Keyframe();
                
                void setFromKF(FrameHessian* fh, CalibHessian HCalib);

                void setFromF(FrameShell* fs, CalibHessian* Hcalib);

                int id;
                bool active;

                SE3 camToWorld;

                inline bool operator < (const Keyframe& other) const
                {
                    return (id < other.id)
                }

             private:
                //calib params?
                float fx, fy, cx, cy;
                float fxi, fyi, cxi, cyi;
                int width, height;

                float my_scaledTH, my_absTH, my_scale;
                int my_sparsifyFactor;
                int my_displayMode;
                float my_minrelBS;
                //bool needRefresh;

                int numSparsePoints;
                int numSparseBufferSize;
                InputPointSparse<MAX_RES_PER_POINT>* originalInputSparse;

                //bool bufferValid;
                //int numGLBufferPoints;
                //int numGLBufferGoodPoints;
                //pangolin::GlBuffer vertexBuffer;
                //pangolin::GlBuffer colorBuffer;


            
        };

     }
}
