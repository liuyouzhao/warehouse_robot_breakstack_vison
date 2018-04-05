#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

using namespace std;

static void writeHeader(FILE *f, string name, string type, int rows, int cols)
{
    const char *h1 = "# Created by Octave 4.0.0, Wed Mar 14 20:33:03 2018 CST <hujia@hujia>\n";
    const char *h2 = "# name: ";
    const char *h3 = "# type: ";
    const char *h4 = "# rows: ";
    const char *h5 = "# columns: ";

    char charName[64] = {0};
    char charType[64] = {0};
    char charRows[64] = {0};
    char charCols[64] = {0};

    sprintf(charName, "%s%s\r\n", h2, name.c_str());
    sprintf(charType, "%s%s\r\n", h3, type.c_str());
    sprintf(charRows, "%s%d\r\n", h4, rows);
    sprintf(charCols, "%s%d\r\n", h5, cols);

    fwrite(h1, strlen(h1), 1, f);
    fwrite(charName, strlen(charName), 1, f);
    fwrite(charType, strlen(charType), 1, f);
    fwrite(charRows, strlen(charRows), 1, f);
    fwrite(charCols, strlen(charCols), 1, f);
}

void writeX(const char *filename, vector<double> X)
{
    int rows = X.size();
    int cols = 1;
    string name = string("x");
    string type = string("matrix");

    FILE *f = fopen(filename, "w");

    writeHeader(f, name, type, rows, cols);

    for(int i = 0; i < rows; i ++)
    {
        char cNum[32] = {0};
        sprintf(cNum, "%f\n", X[i]);
        fwrite(cNum, strlen(cNum), 1, f);
    }

    fclose(f);
}

void writeOctaveCommon(const char *filename, string name, vector<float> X)
{
    int rows = X.size();
    int cols = 1;
    string type = string("matrix");

    FILE *f = fopen(filename, "w");

    writeHeader(f, name, type, rows, cols);

    for(int i = 0; i < rows; i ++)
    {
        char cNum[32] = {0};
        sprintf(cNum, "%f\n", X[i]);
        fwrite(cNum, strlen(cNum), 1, f);
    }

    fclose(f);
}
