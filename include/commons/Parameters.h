/* 
 * File:   Parameters.h
 * Author: daniele
 *
 * Created on 14 marzo 2015, 18.22
 */

#ifndef PARAMETERS_H
#define	PARAMETERS_H
#include <pcl/console/parse.h>

namespace visy {

    class Parameters {
    public:
        Parameters(int argc, char** argv);
        virtual ~Parameters();
        template<typename T> void readValue(std::string name);
        void putString(std::string name);
        void putFloat(std::string name);
        void putInt(std::string name);
        void putBool(std::string name);
        std::string getString(std::string name);
        float getFloat(std::string name);
        int getInt(std::string name);
        bool getBool(std::string name);
        static std::vector<float> parseFloatArray(std::string s,std::string delimiter = ";");
    private:
        int argc;
        char** argv;
        std::map<std::string, std::string> strings;
        std::map<std::string, float> floats;
        std::map<std::string, int> ints;
        std::map<std::string, bool> bools;
        
    };
}
#endif	/* PARAMETERS_H */

