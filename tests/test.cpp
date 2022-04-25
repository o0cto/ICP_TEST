#include <iostream>
#include <string>
#include <fstream>
#include <unistd.h>

#include <CCCoreLib/MyPointCloud.h>
#include <CCCoreLib/MyMesh.h>
#include <CCCoreLib/RegistrationTools.h>
//local
#include <CCCoreLib/CCMath.h>
#include <CCCoreLib/CloudSamplingTools.h>
#include <CCCoreLib/DistanceComputationTools.h>
#include <CCCoreLib/Garbage.h>
#include <CCCoreLib/GenericProgressCallback.h>
#include <CCCoreLib/GenericIndexedMesh.h>
#include <CCCoreLib/GeometricalAnalysisTools.h>
#include <CCCoreLib/Jacobi.h>
#include <CCCoreLib/KdTree.h>
#include <CCCoreLib/ManualSegmentationTools.h>
#include <CCCoreLib/NormalDistribution.h>
#include <CCCoreLib/ParallelSort.h>
#include <CCCoreLib/PointCloud.h>
#include <CCCoreLib/ReferenceCloud.h>
#include <CCCoreLib/ScalarFieldTools.h>

#include <time.h>
#include <ratio>
#include <chrono>

# define COM_POINTS_NUM 40097
# define REF_POINTS_NUM 35947
# define REF_VERTICES_NUM 69451 

inline bool FileExist (const std::string& name) 
{
    return ( access( name.c_str(), F_OK ) != -1 );
}

bool LoadMyMesh(const std::string &mystr, MyScan::MyMesh *mymesh, const int num);
bool LoadMyPointCloud(const std::string &mystr, MyScan::MyPointCloud * mycloud, const int num);
ICPRegistrationTools::RESULT_TYPE ICPRegister(	GenericIndexedCloudPersist* inputModelCloud,
                                                GenericIndexedMesh* inputModelMesh,
                                                GenericIndexedCloudPersist* inputDataCloud,
                                                const ICPRegistrationTools::Parameters& params,
                                                RegistrationTools::ScaledTransformation& transform,
                                                double& finalRMS,
                                                unsigned& finalPointCount,
                                                GenericProgressCallback* progressCb = nullptr/*=nullptr*/);


int main(int argc, char * argv[])
{
    // 加载点云数据
    MyScan::MyPointCloud *com_cloud = new MyScan::MyPointCloud;
    MyScan::MyPointCloud *ref_cloud = new MyScan::MyPointCloud;

    std::string comFile = "../data/bun045_40097.bin";
    std::string refFile = "../data/bun_zipper_points_35947.bin";
    std::string meshFile = "../data/bun_zipper_vertices_69451.bin";

    if(!LoadMyPointCloud(comFile, com_cloud, COM_POINTS_NUM))
        return (-1);
    if(!LoadMyPointCloud(refFile, ref_cloud, REF_POINTS_NUM))
        return -1;
  
    MyScan::MyMesh * mesh = new MyScan::MyMesh(ref_cloud);
    if(!LoadMyMesh(meshFile, mesh, REF_VERTICES_NUM))
        return (-1);

    CCCoreLib::PointProjectionTools::Transformation transform;
    CCCoreLib::ICPRegistrationTools::Parameters params;
    CCCoreLib::ICPRegistrationTools::RESULT_TYPE result;
    {
        params.minRMSDecrease			    = 1.0E-05;
        params.filterOutFarthestPoints      = true;
        params.samplingLimit			    = 50000;
        params.maxThreadCount			    = 8;
    }
    double finalRMS = 0.0;
    unsigned finalPointCount = 0;


    std::cout << "start ICP, please wait ..." << std::endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    
    result = ICPRegister(ref_cloud, 
                        mesh, 
                        com_cloud,
                        params, 
                        transform, 
                        finalRMS, 
                        finalPointCount);
    std::cout << "ICP result = " <<  result << std::endl;

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Register took " << time_span.count() << " seconds." << std::endl;

    // 判断ICP结果是否正确
    double trans_real[12] = { 0.832533359528, 0.013966505416, 0.553798854351, -0.053866289556,
                            -0.020397977903, 0.999777078629, 0.005450722761, 0.000294588244,
                            -0.553599298000, -0.015834284946, 0.832632660866, -0.010680015199};
    double trans_mat[12] = { transform.R.getValue(0,0), transform.R.getValue(0,1), transform.R.getValue(0,2), transform.T.x,
                            transform.R.getValue(1,0), transform.R.getValue(1,1), transform.R.getValue(1,2), transform.T.y,
                            transform.R.getValue(2,0), transform.R.getValue(2,1), transform.R.getValue(2,2), transform.T.z };
                
    for (int i = 0; i < 12; i ++)  
    {
        if(abs(trans_mat[i] - trans_real[i]) > 1e-5)
            return (-1);
    } 
    std::cout << "ICP success." << std::endl;
    return 0;
}

bool LoadMyMesh(const std::string &mystr, MyScan::MyMesh *mymesh, const int num)
{
    if(!FileExist(mystr) || num <= 0)
        return false;

    unsigned *data = new unsigned[3 * num];
    std::ifstream inF(mystr, std::ios::binary);
    inF.read(reinterpret_cast<char*>(data), sizeof(unsigned) * num * 3);
    inF.close();
    if(data == nullptr)
        return false;

    int res = mymesh->reserve(num);
    for (int i = 0; i < num; i++) {
        mymesh->addTriangle(data[i*3], data[i*3+1], data[i*3+2]);
    }

    delete data;
    return true;
}

bool LoadMyPointCloud(const std::string &mystr, MyScan::MyPointCloud * mycloud, const int num)
{
    if(!FileExist(mystr) || num <= 0)
        return false;

    float * data = new float[3 * num];
    std::ifstream inF(mystr, std::ios::binary);
    inF.read(reinterpret_cast<char*>(data), sizeof(float) * num * 3);
    inF.close();
    if(data == nullptr)
        return false;

    int res = mycloud->reserve(num);
    for (int i = 0; i < num; i++) {
        CCVector3 P11(data[i*3], data[i*3+1], data[i*3+2]);
        mycloud->addPoint(P11);
    }

    delete data;
    return true;
}

struct ModelCloud
{
	ModelCloud() : cloud(nullptr), weights(nullptr) {}
	ModelCloud(const ModelCloud& m) = default;
	GenericIndexedCloudPersist* cloud;
	ScalarField* weights;
};

struct DataCloud
{
	DataCloud() : cloud(nullptr), rotatedCloud(nullptr), weights(nullptr), CPSetRef(nullptr), CPSetPlain(nullptr) {}

	ReferenceCloud* cloud;
	PointCloud* rotatedCloud;
	ScalarField* weights;
	ReferenceCloud* CPSetRef;
	PointCloud* CPSetPlain;
};

ICPRegistrationTools::RESULT_TYPE ICPRegister(	GenericIndexedCloudPersist* inputModelCloud,
                                                GenericIndexedMesh* inputModelMesh,
                                                GenericIndexedCloudPersist* inputDataCloud,
                                                const ICPRegistrationTools::Parameters& params,
                                                RegistrationTools::ScaledTransformation& transform,
                                                double& finalRMS,
                                                unsigned& finalPointCount,
                                                GenericProgressCallback* progressCb/*=nullptr*/)
{
	if (!inputModelCloud || !inputDataCloud)
	{
		assert(false);
		return ICPRegistrationTools::ICP_ERROR_INVALID_INPUT;
	}

	//hopefully the user will understand it's not possible ;)
	finalRMS = -1.0;

	Garbage<GenericIndexedCloudPersist> cloudGarbage;
	Garbage<ScalarField> sfGarbage;

	bool registerWithNormals = (params.normalsMatching != ICPRegistrationTools::NO_NORMAL);

	//DATA CLOUD (will move)
	DataCloud data;
	{
		//we also want to use the same number of points for registration as initially defined by the user!
		unsigned dataSamplingLimit = params.finalOverlapRatio != 1.0 ? static_cast<unsigned>(params.samplingLimit / params.finalOverlapRatio) : params.samplingLimit;

		if (inputDataCloud->size() == 0)
		{
			return ICPRegistrationTools::ICP_NOTHING_TO_DO;
		}
		else if (inputDataCloud->size() > dataSamplingLimit)
		{
			//we resample the cloud if it's too big (speed increase)
			data.cloud = CloudSamplingTools::subsampleCloudRandomly(inputDataCloud, dataSamplingLimit);
			if (!data.cloud)
			{
				return ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
			}
			cloudGarbage.add(data.cloud);

			//if we need to resample the weights as well
			if (params.dataWeights)
			{
				data.weights = new ScalarField("ResampledDataWeights");
				sfGarbage.add(data.weights);

				unsigned destCount = data.cloud->size();
				if (data.weights->resizeSafe(destCount))
				{
					for (unsigned i = 0; i < destCount; ++i)
					{
						unsigned pointIndex = data.cloud->getPointGlobalIndex(i);
						data.weights->setValue(i, params.dataWeights->getValue(pointIndex));
					}
					data.weights->computeMinAndMax();
				}
				else
				{
					//not enough memory
					return ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
				}
			}
		}
		else //no need to resample
		{
			//we still create a 'fake' reference cloud with all the points
			data.cloud = new ReferenceCloud(inputDataCloud);
			cloudGarbage.add(data.cloud);
			if (!data.cloud->addPointIndex(0, inputDataCloud->size()))
			{
				//not enough memory
				return ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
			}
			if (params.dataWeights)
			{
				//we use the input weights
				data.weights = new ScalarField(*params.dataWeights);
				sfGarbage.add(data.weights);
			}
		}

		//eventually we'll need a scalar field on the data cloud
		if (!data.cloud->enableScalarField())
		{
			//not enough memory
			return ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
		}

		//we need normals to register with normals ;)
		registerWithNormals &= inputDataCloud->normalsAvailable();
	}
	assert(data.cloud);

	//octree level for cloud/mesh distances computation
	unsigned char meshDistOctreeLevel = 8;

	//MODEL ENTITY (reference, won't move)
	ModelCloud model;
	if (inputModelMesh)
	{
		assert(!params.modelWeights);

		if (inputModelMesh->size() == 0)
		{
			return ICPRegistrationTools::ICP_ERROR_INVALID_INPUT;
		}

		//we'll use the mesh vertices to estimate the right octree level
		DgmOctree dataOctree(data.cloud);
		DgmOctree modelOctree(inputModelCloud);
		if (dataOctree.build() < static_cast<int>(data.cloud->size()) || modelOctree.build() < static_cast<int>(inputModelCloud->size()))
		{
			//an error occurred during the octree computation: probably there's not enough memory
			return ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
		}

		meshDistOctreeLevel = dataOctree.findBestLevelForComparisonWithOctree(&modelOctree);

		//we need normals to register with normals ;)
		registerWithNormals &= inputModelMesh->normalsAvailable();
	}
	else /*if (inputModelCloud)*/
	{
		if (inputModelCloud->size() == 0)
		{
			return ICPRegistrationTools::ICP_ERROR_INVALID_INPUT;
		}
		else if (inputModelCloud->size() > params.samplingLimit)
		{
			//we resample the cloud if it's too big (speed increase)
			ReferenceCloud* subModelCloud = CloudSamplingTools::subsampleCloudRandomly(inputModelCloud, params.samplingLimit);
			if (!subModelCloud)
			{
				//not enough memory
				return ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
			}
			cloudGarbage.add(subModelCloud);

			//if we need to resample the weights as well
			if (params.modelWeights)
			{
				model.weights = new ScalarField("ResampledModelWeights");
				sfGarbage.add(model.weights);

				unsigned destCount = subModelCloud->size();
				if (model.weights->resizeSafe(destCount))
				{
					for (unsigned i = 0; i < destCount; ++i)
					{
						unsigned pointIndex = subModelCloud->getPointGlobalIndex(i);
						model.weights->setValue(i, params.modelWeights->getValue(pointIndex));
					}
					model.weights->computeMinAndMax();
				}
				else
				{
					//not enough memory
					return ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
				}
			}
			model.cloud = subModelCloud;
		}
		else
		{
			//we use the input cloud and weights
			model.cloud = inputModelCloud;
			model.weights = params.modelWeights;
		}
		assert(model.cloud);

		//we need normals to register with normals ;)
		registerWithNormals &= inputModelCloud->normalsAvailable();
	}

	//for partial overlap
	unsigned maxOverlapCount = 0;
	std::vector<float> overlapDistances;
	if (params.finalOverlapRatio < 1.0)
	{
		//we pre-allocate the memory to sort distance values later
		try
		{
			overlapDistances.resize(data.cloud->size());
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
		}
		maxOverlapCount = static_cast<unsigned>(params.finalOverlapRatio*data.cloud->size());
		assert(maxOverlapCount != 0);
	}

	//Closest Point Set (see ICP algorithm)
	if (inputModelMesh)
	{
		data.CPSetPlain = new PointCloud;
		cloudGarbage.add(data.CPSetPlain);
	}
	else
	{
		data.CPSetRef = new ReferenceCloud(model.cloud);
		cloudGarbage.add(data.CPSetRef);
	}

	//per-point couple weights
	ScalarField* coupleWeights = nullptr;
	if (model.weights || data.weights || registerWithNormals)
	{
		coupleWeights = new ScalarField("CoupleWeights");
		sfGarbage.add(coupleWeights);
	}

	//we compute the initial distance between the two clouds (and the CPSet by the way)
	//data.cloud->forEach(ScalarFieldTools::SetScalarValueToNaN); //DGM: done automatically in computeCloud2CloudDistances now
	if (inputModelMesh)
	{
		assert(data.CPSetPlain);
		DistanceComputationTools::Cloud2MeshDistancesComputationParams c2mDistParams;
		c2mDistParams.octreeLevel = meshDistOctreeLevel;
		c2mDistParams.signedDistances = params.useC2MSignedDistances;
		c2mDistParams.CPSet = data.CPSetPlain;
		c2mDistParams.maxThreadCount = params.maxThreadCount;
		if (DistanceComputationTools::computeCloud2MeshDistances(data.cloud, inputModelMesh, c2mDistParams, progressCb) < 0)
		{
			//an error occurred during distances computation...
			return ICPRegistrationTools::ICP_ERROR_DIST_COMPUTATION;
		}
	}
	else if (inputModelCloud)
	{
		assert(data.CPSetRef);
		DistanceComputationTools::Cloud2CloudDistancesComputationParams c2cDistParams;
		c2cDistParams.CPSet = data.CPSetRef;
		c2cDistParams.maxThreadCount = params.maxThreadCount;
		if (DistanceComputationTools::computeCloud2CloudDistances(data.cloud, model.cloud, c2cDistParams, progressCb) < 0)
		{
			//an error occurred during distances computation...
			return ICPRegistrationTools::ICP_ERROR_DIST_COMPUTATION;
		}
	}
	else
	{
		assert(false);
	}

	FILE* fTraceFile = nullptr;
#ifdef CC_DEBUG
	fTraceFile = fopen("registration_trace_log.csv", "wt");
#endif
	if (fTraceFile)
	{
		fprintf(fTraceFile, "Iteration; RMS; Point count;\n");
	}

	double lastStepRMS = -1.0;
	double initialDeltaRMS = -1.0;
	RegistrationTools::ScaledTransformation currentTrans;
	ICPRegistrationTools::RESULT_TYPE result = ICPRegistrationTools::ICP_ERROR;

	for (unsigned iteration = 0;; ++iteration)
	{
        fprintf(stderr, "start: %u;\n", iteration);
		if (progressCb && progressCb->isCancelRequested())
		{
			result = ICPRegistrationTools::ICP_ERROR_CANCELED_BY_USER;
			break;
		}

		//shall we remove the farthest points?
		bool pointOrderHasBeenChanged = false;
		if (params.filterOutFarthestPoints)
		{
			NormalDistribution N;
			N.computeParameters(data.cloud);
			if (N.isValid())
			{
				float mu;
				float sigma2;
				N.getParameters(mu, sigma2);
				float maxDistance = static_cast<float>(mu + 2.5*sqrt(sigma2));

				DataCloud filteredData;
				filteredData.cloud = new ReferenceCloud(data.cloud->getAssociatedCloud());
				cloudGarbage.add(filteredData.cloud);

				if (data.CPSetRef)
				{
					filteredData.CPSetRef = new ReferenceCloud(data.CPSetRef->getAssociatedCloud()); //we must also update the CPSet!
					cloudGarbage.add(filteredData.CPSetRef);
				}
				else if (data.CPSetPlain)
				{
					filteredData.CPSetPlain = new PointCloud; //we must also update the CPSet!
					cloudGarbage.add(filteredData.CPSetPlain);
				}

				if (data.weights)
				{
					filteredData.weights = new ScalarField("ResampledDataWeights");
					sfGarbage.add(filteredData.weights);
				}

				unsigned pointCount = data.cloud->size();
				if (!filteredData.cloud->reserve(pointCount)
					|| (filteredData.CPSetRef && !filteredData.CPSetRef->reserve(pointCount))
					|| (filteredData.CPSetPlain && !filteredData.CPSetPlain->reserve(pointCount))
					|| (filteredData.weights && !filteredData.weights->reserveSafe(pointCount)))
				{
					//not enough memory
					result = ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
					break;
				}

				//we keep only the points with "not too high" distances
				for (unsigned i = 0; i < pointCount; ++i)
				{
					if (data.cloud->getPointScalarValue(i) <= maxDistance)
					{
						filteredData.cloud->addPointIndex(data.cloud->getPointGlobalIndex(i));
						if (filteredData.CPSetRef)
							filteredData.CPSetRef->addPointIndex(data.CPSetRef->getPointGlobalIndex(i));
						else if (filteredData.CPSetPlain)
							filteredData.CPSetPlain->addPoint(*(data.CPSetPlain->getPoint(i)));
						if (filteredData.weights)
							filteredData.weights->addElement(data.weights->getValue(i));
					}
				}

				//resize should be ok as we have called reserve first
				filteredData.cloud->resize(filteredData.cloud->size()); //should always be ok as current size < pointCount
				if (filteredData.CPSetRef)
					filteredData.CPSetRef->resize(filteredData.CPSetRef->size());
				else if (filteredData.CPSetPlain)
					filteredData.CPSetPlain->resize(filteredData.CPSetPlain->size());
				if (filteredData.weights)
					filteredData.weights->resize(filteredData.weights->currentSize());

				//replace old structures by new ones
				cloudGarbage.destroy(data.cloud);
				if (data.CPSetRef)
					cloudGarbage.destroy(data.CPSetRef);
				else if (data.CPSetPlain)
					cloudGarbage.destroy(data.CPSetPlain);
				if (data.weights)
					sfGarbage.destroy(data.weights);
				data = filteredData;

				pointOrderHasBeenChanged = true;
			}
		}

		//shall we ignore/remove some points based on their distance?
		DataCloud trueData;
		unsigned pointCount = data.cloud->size();
		if (maxOverlapCount != 0 && pointCount > maxOverlapCount)
		{
			assert(overlapDistances.size() >= pointCount);
			for (unsigned i = 0; i < pointCount; ++i)
			{
				overlapDistances[i] = data.cloud->getPointScalarValue(i);
				assert(overlapDistances[i] == overlapDistances[i]);
			}

			ParallelSort(overlapDistances.begin(), overlapDistances.begin() + pointCount);

			assert(maxOverlapCount != 0);
			float maxOverlapDist = overlapDistances[maxOverlapCount - 1];

			DataCloud filteredData;
			filteredData.cloud = new ReferenceCloud(data.cloud->getAssociatedCloud());
			if (data.CPSetRef)
			{
				filteredData.CPSetRef = new ReferenceCloud(data.CPSetRef->getAssociatedCloud()); //we must also update the CPSet!
				cloudGarbage.add(filteredData.CPSetRef);
			}
			else if (data.CPSetPlain)
			{
				filteredData.CPSetPlain = new PointCloud; //we must also update the CPSet!
				cloudGarbage.add(filteredData.CPSetPlain);
			}
			cloudGarbage.add(filteredData.cloud);
			if (data.weights)
			{
				filteredData.weights = new ScalarField("ResampledDataWeights");
				sfGarbage.add(filteredData.weights);
			}

			if (!filteredData.cloud->reserve(pointCount) //should be maxOverlapCount in theory, but there may be several points with the same value as maxOverlapDist!
				|| (filteredData.CPSetRef && !filteredData.CPSetRef->reserve(pointCount))
				|| (filteredData.CPSetPlain && !filteredData.CPSetPlain->reserve(pointCount))
				|| (filteredData.CPSetPlain && !filteredData.CPSetPlain->enableScalarField()) //don't forget the scalar field with the nearest triangle index
				|| (filteredData.weights && !filteredData.weights->reserveSafe(pointCount)))
			{
				//not enough memory
				result = ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}

			//we keep only the points with "not too high" distances
			for (unsigned i = 0; i < pointCount; ++i)
			{
				if (data.cloud->getPointScalarValue(i) <= maxOverlapDist)
				{
					filteredData.cloud->addPointIndex(data.cloud->getPointGlobalIndex(i));
					if (filteredData.CPSetRef)
					{
						filteredData.CPSetRef->addPointIndex(data.CPSetRef->getPointGlobalIndex(i));
					}
					else if (filteredData.CPSetPlain)
					{
						filteredData.CPSetPlain->addPoint(*(data.CPSetPlain->getPoint(i)));
						//don't forget the scalar field with the nearest triangle index!
						filteredData.CPSetPlain->addPointScalarValue(data.CPSetPlain->getPointScalarValue(i));
					}
					if (filteredData.weights)
					{
						filteredData.weights->addElement(data.weights->getValue(i));
					}
				}
			}
			assert(filteredData.cloud->size() >= maxOverlapCount);

			//resize should be ok as we have called reserve first
			filteredData.cloud->resize(filteredData.cloud->size()); //should always be ok as current size < pointCount
			if (filteredData.CPSetRef)
				filteredData.CPSetRef->resize(filteredData.CPSetRef->size());
			else if (filteredData.CPSetPlain)
				filteredData.CPSetPlain->resize(filteredData.CPSetPlain->size());
			if (filteredData.weights)
				filteredData.weights->resize(filteredData.weights->currentSize());

			//(temporarily) replace old structures by new ones
			trueData = data;
			data = filteredData;
		}

		//update couple weights (if any)
		if (coupleWeights)
		{
			unsigned count = data.cloud->size();
			assert(model.weights || data.weights || registerWithNormals);
			assert(!model.weights || (data.CPSetRef && data.CPSetRef->size() == count));

			if (coupleWeights->currentSize() != count && !coupleWeights->resizeSafe(count))
			{
				//not enough memory to store weights
				result = ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}

			for (unsigned i = 0; i < count; ++i)
			{
				double w = 1.0;
				if (registerWithNormals)
				{
					//retrieve the data point normal
					const CCVector3* Nd = data.cloud->getNormal(i);
					
					//retrieve the nearest model point normal
					CCVector3 Nm;
					if (inputModelMesh)
					{
						unsigned triIndex = static_cast<unsigned>(data.CPSetPlain->getPointScalarValue(i));
						assert(triIndex >= 0 && triIndex < inputModelMesh->size());
						inputModelMesh->interpolateNormals(triIndex, *data.cloud->getPoint(i), Nm);
					}
					else
					{
						Nm = *inputModelCloud->getNormal(i);
					}

					//we assume the vectors are unitary!
					PointCoordinateType dp = Nd->dot(Nm);

					switch (params.normalsMatching)
					{
					case ICPRegistrationTools::OPPOSITE_NORMALS:
					{
						w = acos(dp) / M_PI; // 0 rad --> w = 0 / pi/2 rad --> w = 0.5 / pi rad --> w = 1
					}
					break;

					case ICPRegistrationTools::SAME_SIDE_NORMALS:
					{
						w = 1.0 - acos(dp) / M_PI; // 0 rad --> w = 1 / pi/2 rad --> w = 0.5 / pi rad --> w = 0
					}
					break;

					case ICPRegistrationTools::DOUBLE_SIDED_NORMALS:
					{
						dp = std::abs(dp);
						w = 1.0 - acos(dp) / M_PI_2; // 0 rad --> w = 1 / pi/2 rad --> w = 0
					}
					break;

					default:
						assert(false);
						break;
					}
				}
				if (data.weights)
				{
					w *= data.weights->getValue(i);
				}
				if (model.weights)
				{
					//model weights are only supported with a reference cloud!
					float wm = model.weights->getValue(data.CPSetRef->getPointGlobalIndex(i));
					w *= wm;
				}
				coupleWeights->setValue(i, static_cast<float>(w));
			}
			coupleWeights->computeMinAndMax();
		}

		//we can now compute the best registration transformation for this step
		//(now that we have selected the points that will be used for registration!)
		{
			//if we use weights, we have to compute weighted RMS!!!
			double meanSquareValue = 0.0;
			double wiSum = 0.0; //we normalize the weights by their sum

			for (unsigned i = 0; i < data.cloud->size(); ++i)
			{
				float V = data.cloud->getPointScalarValue(i);
				if (ScalarField::ValidValue(V))
				{
					double wi = 1.0;
					if (coupleWeights)
					{
						float w = coupleWeights->getValue(i);
						if (!ScalarField::ValidValue(w))
							continue;
						wi = std::abs(w);
					}
					double Vd = wi * V;
					wiSum += wi * wi;
					meanSquareValue += Vd * Vd;
				}
			}

			//12/11/2008 - A.BEY: ICP guarantees only the decrease of the squared distances sum (not the distances sum)
			double meanSquareError = (wiSum != 0 ? static_cast<float>(meanSquareValue / wiSum) : 0);

			double rms = sqrt(meanSquareError);
			
            fprintf(stderr, "end:%u; %f; %u;\n", iteration, rms, data.cloud->size());

			if (iteration == 0)
			{
				//progress notification
				if (progressCb)
				{
					//on the first iteration, we init/show the dialog
					if (progressCb->textCanBeEdited())
					{
						progressCb->setMethodTitle("Clouds registration");
						char buffer[256];
						sprintf(buffer, "Initial RMS = %f\n", rms);
						progressCb->setInfo(buffer);
					}
					progressCb->update(0);
					progressCb->start();
				}

				finalRMS = rms;
				finalPointCount = data.cloud->size();

				if (LessThanEpsilon(rms))
				{
					//nothing to do
					result = ICPRegistrationTools::ICP_NOTHING_TO_DO;
					break;
				}
			}
			else
			{
				assert(lastStepRMS >= 0.0);

				if (rms > lastStepRMS) //error increase!
				{
					result = iteration == 1 ? ICPRegistrationTools::ICP_NOTHING_TO_DO : ICPRegistrationTools::ICP_APPLY_TRANSFO;
					break;
				}

				//error update (RMS)
				double deltaRMS = lastStepRMS - rms;
				//should be better!
				assert(deltaRMS >= 0.0);

				//we update the global transformation matrix
				if (currentTrans.R.isValid())
				{
					if (transform.R.isValid())
						transform.R = currentTrans.R * transform.R;
					else
						transform.R = currentTrans.R;

					transform.T = currentTrans.R * transform.T;
				}

				if (params.adjustScale)
				{
					transform.s *= currentTrans.s;
					transform.T *= currentTrans.s;
				}

				transform.T += currentTrans.T;

				finalRMS = rms;
				finalPointCount = data.cloud->size();

				//stop criterion
				if (	(params.convType == ICPRegistrationTools::MAX_ERROR_CONVERGENCE && deltaRMS < params.minRMSDecrease) //convergence reached
						||	(params.convType == ICPRegistrationTools::MAX_ITER_CONVERGENCE && iteration >= params.nbMaxIterations) //max iteration reached
						)
				{
					result = ICPRegistrationTools::ICP_APPLY_TRANSFO;
					break;
				}

				//progress notification
				if (progressCb)
				{
					if (progressCb->textCanBeEdited())
					{
						char buffer[256];
						if (coupleWeights)
							sprintf(buffer, "Weighted RMS = %f [-%f]\n", rms, deltaRMS);
						else
							sprintf(buffer, "RMS = %f [-%f]\n", rms, deltaRMS);
						progressCb->setInfo(buffer);
					}
					if (iteration == 1)
					{
						initialDeltaRMS = deltaRMS;
						progressCb->update(0);
					}
					else
					{
						assert(initialDeltaRMS >= 0.0);
						float progressPercent = static_cast<float>((initialDeltaRMS - deltaRMS) / (initialDeltaRMS - params.minRMSDecrease)*100.0);
						progressCb->update(progressPercent);
					}
				}
			}

			lastStepRMS = rms;
		}

		//single iteration of the registration procedure
		currentTrans = RegistrationTools::ScaledTransformation();
		if (!RegistrationTools::RegistrationProcedure(	data.cloud,
														data.CPSetRef ? static_cast<GenericCloud*>(data.CPSetRef) : static_cast<GenericCloud*>(data.CPSetPlain),
														currentTrans,
														params.adjustScale,
														coupleWeights))
		{
			result = ICPRegistrationTools::ICP_ERROR_REGISTRATION_STEP;
			break;
		}

		//restore original data sets (if any were stored)
		if (trueData.cloud)
		{
			cloudGarbage.destroy(data.cloud);
			if (data.CPSetRef)
				cloudGarbage.destroy(data.CPSetRef);
			else if (data.CPSetPlain)
				cloudGarbage.destroy(data.CPSetPlain);
			if (data.weights)
				sfGarbage.destroy(data.weights);
			data = trueData;
		}

		//shall we filter some components of the resulting transformation?
		if (params.transformationFilters != RegistrationTools::SKIP_NONE)
		{
			//filter translation (in place)
			RegistrationTools::FilterTransformation(currentTrans, params.transformationFilters, currentTrans);
		}

		//get rotated data cloud
		if (!data.rotatedCloud || pointOrderHasBeenChanged)
		{
			//we create a new structure, with rotated points
			PointCloud* rotatedDataCloud = PointProjectionTools::applyTransformation(data.cloud, currentTrans);
			if (!rotatedDataCloud)
			{
				//not enough memory
				result = ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}
			//replace data.rotatedCloud
			if (data.rotatedCloud)
				cloudGarbage.destroy(data.rotatedCloud);
			data.rotatedCloud = rotatedDataCloud;
			cloudGarbage.add(data.rotatedCloud);

			//update data.cloud
			data.cloud->clear();
			data.cloud->setAssociatedCloud(data.rotatedCloud);
			if (!data.cloud->addPointIndex(0, data.rotatedCloud->size()))
			{
				//not enough memory
				result = ICPRegistrationTools::ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}
		}
		else
		{
			//we simply have to rotate the existing temporary cloud
			currentTrans.apply(*data.rotatedCloud);
			data.rotatedCloud->invalidateBoundingBox(); //invalidate bb

			//DGM: warning, we must manually invalidate the ReferenceCloud bbox after rotation!
			data.cloud->invalidateBoundingBox();
		}

		//compute (new) distances to model
		if (inputModelMesh)
		{
			DistanceComputationTools::Cloud2MeshDistancesComputationParams c2mDistParams;
			c2mDistParams.octreeLevel = meshDistOctreeLevel;
			c2mDistParams.signedDistances = params.useC2MSignedDistances;
			c2mDistParams.CPSet = data.CPSetPlain;
			c2mDistParams.maxThreadCount = params.maxThreadCount;
			if (DistanceComputationTools::computeCloud2MeshDistances(data.cloud, inputModelMesh, c2mDistParams) < 0)
			{
				//an error occurred during distances computation...
				result = ICPRegistrationTools::ICP_ERROR_REGISTRATION_STEP;
				break;
			}
		}
		else if (inputDataCloud)
		{
			DistanceComputationTools::Cloud2CloudDistancesComputationParams c2cDistParams;
			c2cDistParams.CPSet = data.CPSetRef;
			c2cDistParams.maxThreadCount = params.maxThreadCount;
			if (DistanceComputationTools::computeCloud2CloudDistances(data.cloud, model.cloud, c2cDistParams) < 0)
			{
				//an error occurred during distances computation...
				result = ICPRegistrationTools::ICP_ERROR_REGISTRATION_STEP;
				break;
			}
		}
		else
		{
			assert(false);
		}
	}

	//end of tracefile
	if (fTraceFile)
	{
		fclose(fTraceFile);
		fTraceFile = nullptr;
	}

	//end of progress notification
	if (progressCb)
	{
		progressCb->stop();
	}

	return result;
}

