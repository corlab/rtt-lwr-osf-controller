#pragma once

#include <eigen3/Eigen/Core>
#include <iostream>
#include <fstream>
#include <stdlib.h> //getenv
#include <string>

class FileWriterCSV {
public:
	FileWriterCSV() { };
	FileWriterCSV(std::string fname);
	~FileWriterCSV();

    void openFile(std::string fname);
    void closeFile();
    void writeFile(Eigen::VectorXd & vector);
protected:
    std::string home;
    std::ofstream myfile;
};

