#include <stdio.h>
#include "util/settings.h"

#include "Keyframe.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/ImmaturePoint.h"
#include "util/FrameShell.h"

namespace dso
{
    namespace IOWrap
    {
        Keyframe::Keyframe()
        {

            originalInputSparse = 0;
            numSparseBufferSize = 0;
            numSparsePoints = 0;

            id = 0;
            active=true;
            camToWorld=SE3();

            //needRefresh=true;
            
            my_scaledTh=1e10;
            my_absTh=1e10;
            my_displayMode=1;
            my_minrelBS=1;
            my_sparsityFactor=1;

            //numGLBufferPoints=0;
            //bufferValid = false;
        }
        void KeyFrameDisplay::setfromF(FrameShell* frame, CalibHessian* HCalib)
        {
           id = frame->id;
           fx = HCalib->fxl();
           fy = HCalib->fyl();
           cx = HCalib->cxl();
           cy = HCalib->cyl();
           width = wG[0];
           height = hG[0];
           fxi = 1/fx;
           fyi = 1/fy;
           cxi = -cx / fx;
           cyi = - cy / fy;
           camToWorld = frame->camToWorld;
           //needRefresh=true;
        
        }

        void Keyframe::setFromKF(FrameHessian* fh, CalibHessian* HCalib)
        {
            setfromF(fh->shell, HCalib);
            // add all traces, inlier and outlier points.
        	int npoints = 	fh->immaturePoints.size() +
    					fh->pointHessians.size() +
    					fh->pointHessiansMarginalized.size() +
	    				fh->pointHessiansOut.size();

        	if(numSparseBufferSize < npoints)
        	{
         		if(originalInputSparse != 0) delete originalInputSparse;
        		numSparseBufferSize = npoints+100;
                originalInputSparse = new InputPointSparse<MAX_RES_PER_POINT>[numSparseBufferSize];
    	    }

            InputPointSparse<MAX_RES_PER_POINT>* pc = originalInputSparse;
         	numSparsePoints=0;
        	for(ImmaturePoint* p : fh->immaturePoints)
         	{
        		for(int i=0;i<patternNum;i++)
    	    		pc[numSparsePoints].color[i] = p->color[i];
    
           		pc[numSparsePoints].u = p->u;
    	    	pc[numSparsePoints].v = p->v;
    	    	pc[numSparsePoints].idpeth = (p->idepth_max+p->idepth_min)*0.5f;
    	    	pc[numSparsePoints].idepth_hessian = 1000;
    	    	pc[numSparsePoints].relObsBaseline = 0;
    		    pc[numSparsePoints].numGoodRes = 1;
        		pc[numSparsePoints].status = 0;
        		numSparsePoints++;
        	}
     
         	for(PointHessian* p : fh->pointHessians)
        	{
	         	for(int i=0;i<patternNum;i++)
	    	    	pc[numSparsePoints].color[i] = p->color[i];
    	    	pc[numSparsePoints].u = p->u;
    	    	pc[numSparsePoints].v = p->v;
    	    	pc[numSparsePoints].idpeth = p->idepth_scaled;
    	    	pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
    	    	pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
    	    	pc[numSparsePoints].numGoodRes =  0;
    	    	pc[numSparsePoints].status=1;
    
    		    numSparsePoints++;
    	    }
     
          	for(PointHessian* p : fh->pointHessiansMarginalized)
        	{
        		for(int i=0;i<patternNum;i++)
    	    		pc[numSparsePoints].color[i] = p->color[i];
    		    pc[numSparsePoints].u = p->u;
    	    	pc[numSparsePoints].v = p->v;
    	    	pc[numSparsePoints].idpeth = p->idepth_scaled;
    	    	pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
    		    pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
	         	pc[numSparsePoints].numGoodRes =  0;
    	    	pc[numSparsePoints].status=2;
    	    	numSparsePoints++;
        	}
    
    	    for(PointHessian* p : fh->pointHessiansOut)
    	    {
    	    	for(int i=0;i<patternNum;i++)
    	    		pc[numSparsePoints].color[i] = p->color[i];
    	    	pc[numSparsePoints].u = p->u;
    		    pc[numSparsePoints].v = p->v;
    		    pc[numSparsePoints].idpeth = p->idepth_scaled;
           		pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
        		pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
    	    	pc[numSparsePoints].numGoodRes =  0;
    	    	pc[numSparsePoints].status=3;
    	    	numSparsePoints++;
    	    }
	        assert(numSparsePoints <= npoints);

        	camToWorld = fh->PRE_camToWorld;
        	needRefresh=true;
        }

        Keyframe::~Keyframe()
        {
            if(originalInputSparse != 0)
                delete[] originalInputSparse;
        }

        


    }


}
