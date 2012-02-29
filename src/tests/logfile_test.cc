#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <math.h>

#include "MainApp.h"

void MainApp::run()
{
		cout << "Running LogFile test" << endl;
        using std::vector;

        vector<float> norms(8);

        LogFile *log = LogFile::getInstance();

        boost::this_thread::at_thread_exit(cleanup());

        log->logHeader(heli::LOG_NORMALIZED_OUTPUTS, "CH1(us)\tCH2(us)\tCH3(us)\tCH4(us)\tCH5(us)\tCH6(us)\tCH7(us)\tCH8(us)");
        for (int i=0; i<10; i++)
        {
                BOOST_FOREACH(float & norm, norms)
                {
                        norm = rand() % 1000 + 1000;
                }
                log->logData(heli::LOG_NORMALIZED_OUTPUTS, norms);
                boost::this_thread::sleep(boost::posix_time::seconds(3));
        }
        cout << "finished loop" << endl;

        stringstream ss;
        ss << __FILE__ << __LINE__ << " error log start";
        log->logMessage("error", ss.str());
        log->logMessage("error", "this is an error");

}
