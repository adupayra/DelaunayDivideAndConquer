#include "Helpers.h"
#include <fstream>

void ExportToCsv(std::string filename, const std::vector<Edge>& edgesToExport)
{
    std::ofstream file("..\\..\\..\\..\\..\\PythonTests\\" + filename + ".csv");

    if (!file.is_open())
    {
        printf("Error");
        return;
    }

    for (size_t i = 0; i < edgesToExport.size(); ++i)
    {
        file << edgesToExport[i].start.x << ";" << edgesToExport[i].start.y << std::endl;
        file << edgesToExport[i].end.x << ";" << edgesToExport[i].end.y << std::endl;
    }
    file.close();
}