#include <thread>
#include <functional>
#include "CarPlanner6d.h"
#include "threadpool.hpp"

std::mutex m_resultLock;
std::vector<Eigen::Matrix<double,8,1> >m_vResults;
std::vector<bool> m_vFailures;
unsigned int nTotalNum;
CarPlanner6d planner;
unsigned int currentItem = 0;

void SetResult(int index, Eigen::Matrix<double,8,1> val)
{
    //boost::mutex::scoped_lock loc(m_resultLock);
    m_vResults[index] = val;
}

void CalculateCubic(Eigen::VectorPose dStartPose,double bi, double ti, double ci, int index)
{
//    double xi = cos(bi);
//    double yi = sin(bi);

//    Eigen::VectorPose dGoalPose;
//    dGoalPose << xi,yi,ti,0,0;

//    //try to solve this cubic
//    Eigen::VectorCubic2D dResult;
//    bool res = planner._Solve2dCubic(dStartPose,dGoalPose,Eigen::Matrix3d::Identity(),ci,dResult,true);
//    double norm = (dGoalPose.head(3) - CubicFunc( dResult, ci ).head(3)).norm();
//    double percentage = ((double)index/(double)nTotalNum)*100.0;
//    Eigen::Matrix<double,8,1> result;
//    if( norm <= 1e-1 ) {
//        //dout("[" << percentage << "%] ***Success in solving [" << dGoalPose.transpose() << "] with ci " << ci << " with norm " << norm);
//        result << bi,ti,ci,dResult[0],dResult[1],dResult[2],dResult[3], 1.0;
//    }else{
//        //dout("[" << percentage << "%] Failure in solving [" << dGoalPose.transpose() << "] with ci " << ci << " with norm " << norm);
//        result << bi,ti,ci,0,0,0,0 ,0;
//    }

//    {
//        boost::mutex::scoped_lock loc(m_resultLock);
//        currentItem++;
//        //now register this item
//        SetResult(index,result);
//    }
}

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    //boost::threadpool::pool threadPool;
    threadPool.size_controller().resize(8);



    //double xMin = 1;
    //double xMax = 11;//11;
    //double xStep = 0.2;//0.2;

    //double yMin = -5;
    //double yMax = 5;
    //double yStep = 0.2;//0.2;

    double biMin = -M_PI;
    double biMax = M_PI;
    double biStep = M_PI/36; //one step every 5 degrees

    double tiMin = -M_PI*8/16;
    double tiMax = M_PI*8/16;
    double tiStep = M_PI/16;

    double ciMin = -3;
    double ciMax = 3;
    double ciStep = 0.25;//0.25;

    //calcualte total number of items to process
    //nTotalNum = ((xMax - xMin)/xStep + 1)*((yMax - yMin)/yStep + 1)*((tiMax - tiMin)/tiStep + 1)*((ciMax - ciMin)/ciStep + 1);
    nTotalNum = ((biMax - biMin)/biStep + 1)*((tiMax - tiMin)/tiStep + 1)*((ciMax - ciMin)/ciStep + 1);

    dout("Calculating a total of " << nTotalNum << " items.");


    m_vResults.resize(nTotalNum);
    m_vFailures.resize(nTotalNum);

    double firstTime = Tic();
    double lastTime = Tic();


    int index = 0;
    //for(double xi = xMin; xi <= xMax ; xi += xStep) {
    //    for(double yi = yMin; yi <= yMax ; yi += yStep) {
    for(double bi = biMin; bi <= biMax ; bi += biStep) {
            for(double ti = tiMin; ti <= tiMax ; ti += tiStep) {
                int startIndex = currentItem;


                //use the original lookup table to get a guess
                //Eigen::VectorPose2D x;
                //x << xi,yi,ti,0;
                //now we have a seed cubic with starting curvature of 0
                Eigen::VectorPose dStartPose = Eigen::VectorPose::Zero();
                //Eigen::VectorPose dGoalPose;

                //calculate the x and y based on bearing

                //dGoalPose << xi,yi,ti,0,0;
                for(double ci = ciMin ; ci <= ciMax ; ci += ciStep) {
                    threadPool.schedule(std::bind(CalculateCubic, dStartPose,bi,ti,ci,index));
                    index++;
                }
            }

            //wait for  this batch to finish
            while(threadPool.pending() > 0){
                if(Toc(lastTime) > 1.0  ){
                    std::unique_lock<std::mutex> loc(m_resultLock, std::try_to_lock);

                    double rate = currentItem/Toc(firstTime);
                    double timeRemaining = (double)(nTotalNum-currentItem)/rate;
                    double percentage = ((double)currentItem/(double)nTotalNum)*100.0;
                    dout("[" << percentage << "%] - (" << currentItem << ") complete. Processing " << rate << " cubics/second - " << timeRemaining << " seconds remaining.");
                    lastTime = Tic();
                }
                usleep(1000);
            }
    }

    dout("Calculation complete.. fixing failures...");
    currentItem = 0;

    //for(double xi = xMin; xi <= xMax ; xi += xStep) {
    //    for(double yi = yMin; yi <= yMax ; yi += yStep) {
    for(double bi = biMin; bi <= biMax ; bi += biStep) {
            for(double ti = tiMin; ti <= tiMax ; ti += tiStep) {
                int startIndex = currentItem;
                for(double ci = ciMin ; ci <= ciMax ; ci += ciStep) {
                   currentItem++;
                }

                if(Toc(lastTime) > 1.0  ){
                    double rate = currentItem/Toc(firstTime);
                    double timeRemaining = (double)(nTotalNum-currentItem)/rate;
                    double percentage = ((double)currentItem/(double)nTotalNum)*100.0;
                    dout("[" << percentage << "%] - (" << currentItem << ") complete. Processing " << rate << " cubics/second - " << timeRemaining << " seconds remaining.");
                    lastTime = Tic();
                }

                //go through all items at this level and figure out every failure
                {
                    //first figure out all the failures
                    for(int ii = startIndex ; ii < currentItem ; ii++){
                        m_vFailures[ii] = (m_vResults[ii](7) == 0);
                    }

                    for(int ii = startIndex ; ii < currentItem ; ii++){
                        if(m_vFailures[ii] == true){
                            //no go through the failures in either direction to find the closest non-failure
                            int downCount = m_vFailures.size()*2, upCount = m_vFailures.size()*2;

                            for(int jj = ii ; jj > 0; jj--){
                                if(m_vFailures[jj] == false){
                                    downCount = jj;
                                    break;
                                }
                            }

                            for(int jj = ii ; jj < m_vFailures.size(); jj++){
                                if(m_vFailures[jj] == false){
                                    upCount = jj;
                                    break;
                                }
                            }

                            if(downCount < upCount) {
                                m_vResults[ii].tail(5) = m_vResults[downCount].tail(5);
                            }else if(upCount < downCount){
                                m_vResults[ii].tail(5) = m_vResults[upCount].tail(5);
                            }else if(upCount == downCount){
                                if(upCount == m_vFailures.size()*2){
                                    dout("ERROR: could not find a non failure case for item with goal: " << m_vResults[ii].head(4).transpose());
                                    assert(false);
                                }else{
                                    m_vResults[ii].tail(5) = m_vResults[downCount].tail(5);
                                }
                            }

                            //signal this is failure
                            m_vResults[ii](7) = 0.0;
                        }
                    }
                }
            }
        //}
    }

    //wait for all threads
    threadPool.wait();

    dout("LUT complete! writing to file...");

    //now go through everything and write it to a file
    std::ofstream file;
    file.open("lut.txt", std::ofstream::trunc);

//    std::string sHeader = boost::str(boost::format("#ifndef _CAR_PLANNER_LUT_H_\n"
//                                                   "#define _CAR_PLANNER_LUT_H_\n"
//                                                   "const double g_xmin=%f;\n"
//                                                   "const double g_xmax=%f;\n"
//                                                   "const double g_xstep=%f;\n"
//                                                   "const double g_ymin=%f;\n"
//                                                   "const double g_ymax=%f;\n"
//                                                   "const double g_ystep=%f;\n"
//                                                   "const double g_tmin=%f;\n"
//                                                   "const double g_tmax=%f;\n"
//                                                   "const double g_tstep=%f;\n"
//                                                   "const double g_cimin=%f;\n"
//                                                   "const double g_cimax=%f;\n"
//                                                   "const double g_cistep=%f;\n"
//                                                   "const double g_CarPlannerLut[%d][8] = {\n")
//                                     % xMin % xMax % xStep % yMin % yMax % yStep
//                                     % tiMin % tiMax % tiStep % ciMin % ciMax % ciStep % m_vResults.size());

std::string sHeader = "Fix sHeader"; //crh

/*
std::string sHeader = boost::str(boost::format("#ifndef _CAR_PLANNER_LUT_H_\n"
                                                   "#define _CAR_PLANNER_LUT_H_\n"
                                                   "const double g_bmin=%f;\n"
                                                   "const double g_bmax=%f;\n"
                                                   "const double g_bstep=%f;\n"
                                                   "const double g_tmin=%f;\n"
                                                   "const double g_tmax=%f;\n"
                                                   "const double g_tstep=%f;\n"
                                                   "const double g_cimin=%f;\n"
                                                   "const double g_cimax=%f;\n"
                                                   "const double g_cistep=%f;\n"
                                                   "const double g_CarPlannerLut[%d][8] = {\n")
                                     % biMin % biMax % biStep
                                     % tiMin % tiMax % tiStep % ciMin % ciMax % ciStep % m_vResults.size());
*/

    file.write(sHeader.c_str(),sHeader.length());

    Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", \t", "\n", "{", "}");
    //and now write each row of the vector
    for(int ii = 0 ; ii < m_vResults.size() ; ii++){
        std::stringstream tmp;
        tmp <<m_vResults[ii].block<7,1>(0,0).transpose().format(CleanFmt) << "," ;

        if(m_vResults[ii](7) == 0.0) {
            tmp << "\t\t //failure";
        }
        tmp << std::endl;
        std::string sRow = tmp.str();
        file.write(sRow.c_str(),sRow.length());
    }

    std::string footer = "}; \n#endif \n";
    file.write(footer.c_str(),footer.length());
    file.close();
}
