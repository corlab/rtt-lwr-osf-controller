#include "FileWriterCSV.hpp"
//#include <rtt/RTT.hpp>


FileWriterCSV::FileWriterCSV(std::string fname) : home(std::string(getenv("HOME"))){
	this->openFile(fname);
}

FileWriterCSV::~FileWriterCSV() {

}

void FileWriterCSV::openFile(std::string fname) {
	myfile.open((home + "/" + fname + ".csv").c_str());
}

void FileWriterCSV::closeFile() {
	myfile.close();
}

void FileWriterCSV::writeFile(Eigen::VectorXd & vector) {
    for (int j = 0; j < vector.size(); j++) {
    	if (j == vector.size()-1) {
    		myfile << vector(j);
    	} else {
    		myfile << vector(j) << ", ";
    	}
    }
    myfile << "\n";
}


