// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
//
// Copyright (c) 2018 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

#include <fstream>
#include "TestReport.h"

using namespace test;
using namespace report;
using namespace std;

Plot_Element::Plot_Element():
m_color("blue"),
m_lineStyle(""),
m_thickness(2.0),
m_arrow(0)
{

}

Plot_Line::Plot_Line(): Plot_Element::Plot_Element()
{
}

Plot_Polygon::Plot_Polygon(): Plot_Element()
{
}

Plot_Polygon::Plot_Polygon(VisiLibity::Polygon x): Plot_Element()
{
    m_polygon = x;
}

Plot_Polygon::Plot_Polygon(VisiLibity::Polygon polygon, int arrow, std::string color, std::string lineStyle)
{
    m_polygon = polygon;
    m_arrow = arrow;
    m_color = color;
    m_lineStyle = lineStyle;
}

Plot::Plot()
{
}

Plot::Plot(std::vector<VisiLibity::Point> x)
{
    m_points = x;
}

Plot::Plot(std::vector<Plot_Line> x)
{
    m_lines = x;
}

Plot::Plot(std::vector<Plot_Polygon> polyList, std::string caption)
{
    m_polygons = polyList;
    m_caption = caption;
}

Report::Report()
{

}

Report::Report(std::string x)
{
    m_testName = x;
    m_content = "% Preamble\n% ---\n\\documentclass{article}\n";
    m_content += "% Packages\n% ---\n\\usepackage{tikz}\n\n\\begin{document}\n";
    m_content += "\\title{Test}\n\\maketitle\n";
    m_content += m_testName + " Test\n";
}

bool Report::render()
{
    this->close();
    ofstream fileOut;
    std::cout << "filename: " << m_testName << '\n';
    fileOut.open(m_testName + "Test.tex");
    fileOut << m_content;
    fileOut.close();
    return true;
}

bool Report::addPlot(Plot x)
{
    m_content += "\\begin{figure}\n\\begin{tikzpicture}[scale=1]\n";

    
    //for each polygon in plot_polygon vector
    for (auto &poly : x.m_polygons)
    {
        m_content += "\\draw [" + poly.m_color + "]";
        if(!poly.m_lineStyle.empty())
        {
            m_content += "[" + poly.m_lineStyle + "]";
        }
        
        if(poly.m_arrow == -1)
        {
            m_content += "[<-]";
        }
        else if(poly.m_arrow == 1)
        {
            m_content += "[->]";
        }
        
        
        //for each point in VisiLibity polygon
        int n = poly.m_polygon.n();
        for (int vertexIdx = 0; vertexIdx < n; vertexIdx++)
             //std::vector<VisiLibity::Point>::iterator it = poly.m_polygon.begin(); it != poly.m_polygon.end(); ++it)
        {

            //TODO: units are centimeters
            m_content += "(" + std::to_string(poly.m_polygon[vertexIdx].x()) + "," + std::to_string(poly.m_polygon[vertexIdx].y()) + ")";
            m_content += "--";
        }
        //the first vertex is also the last vertex
        m_content += "(" + std::to_string(poly.m_polygon[0].x()) + "," + std::to_string(poly.m_polygon[0].y()) + ");\n";
    }
    m_content += "\\end{tikzpicture}\n\\caption{";
    m_content += x.m_caption;
    m_content += "}\n\\end{figure}\n";
    return true;
}

bool Report::close()
{
    m_content += "\\end{document}";
    return true;
}

void Report::addText(std::string x)
{
    m_content += x + '\n';
}
