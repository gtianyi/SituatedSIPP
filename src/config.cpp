#include "config.h"
using namespace tinyxml2;

Config::Config()
{
    loglevel = CN_DEFAULT_LOGLVL;
    allowanyangle = CN_DEFAULT_ALLOWANYANGLE;
    startsafeinterval = CN_DEFAULT_STARTSAFEINTERVAL;
    timelimit = CN_DEFAULT_TIMELIMIT;
    initialprioritization = CN_DEFAULT_INITIALPRIORITIZATION;
    rescheduling = CN_DEFAULT_RESCHEDULING;
    planforturns = CN_DEFAULT_PLANFORTURNS;
    additionalwait = CN_DEFAULT_ADDITIONALWAIT;
    fixedlookahead = CNS_DEFAULT_FIXEDLOOKAHEADLIMIT;
    additionalwait = CNS_DEFAULT_UNITWAITDURATION;
}

bool Config::getConfig(const char* fileName)
{
    std::string value;
    std::stringstream stream;

    XMLDocument doc;
    if(doc.LoadFile(fileName) != XMLError::XML_SUCCESS)
    {
        std::cout << "Error openning config input XML file."<<std::endl;
        return false;
    }

    XMLElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No 'root' element found in XML file."<<std::endl;
        return false;
    }

    XMLElement *algorithm = root->FirstChildElement(CNS_TAG_ALGORITHM);
    if (!algorithm)
    {
        std::cout << "No 'algorithm' element found in XML file."<<std::endl;
        return false;
    }

    XMLElement *element;

    element = algorithm->FirstChildElement(CNS_TAG_ALLOW_AA);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_ALLOW_AA<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set '"<<CNS_DEFAULT_ALLOWANYANGLE<<"'."<<std::endl;
        allowanyangle = true;
    }
    else
    {
        value = element->GetText();
        if(value == "true" || value == "1")
            allowanyangle = true;
        else if(value == "false" || value == "0")
            allowanyangle = false;
        else
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_ALLOW_AA<<"' value. It's set to '"<<CNS_DEFAULT_ALLOWANYANGLE<<"'."<<std::endl;
            allowanyangle = CN_DEFAULT_ALLOWANYANGLE;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_WEIGHT);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_WEIGHT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_WEIGHT<<"'."<<std::endl;
        weight = CN_DEFAULT_WEIGHT;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>weight;
        stream.clear();
        stream.str("");
        if(weight < 1 || weight > 10)
        {
            std::cout << "Warning! Wrong value of '"<<CNS_TAG_WEIGHT<<"' element. Possible values are [1;10] . Its value is set to '"<<CNS_DEFAULT_WEIGHT<<"'."<<std::endl;
            weight = CN_DEFAULT_WEIGHT;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_PRIORITIZATION);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_PRIORITIZATION<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_INITIALPRIORITIZATION<<"'."<<std::endl;
        initialprioritization = CN_DEFAULT_INITIALPRIORITIZATION;
    }
    else
    {
        value = element->GetText();
        if(value == CNS_IP_FIFO)
            initialprioritization = CN_IP_FIFO;
        else if(value == CNS_IP_LONGESTF)
            initialprioritization = CN_IP_LONGESTF;
        else if(value == CNS_IP_SHORTESTF)
            initialprioritization = CN_IP_SHORTESTF;
        else if(value == CNS_IP_RANDOM)
            initialprioritization = CN_IP_RANDOM;
        else
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_PRIORITIZATION<<"' value. It's set to '"<<CNS_DEFAULT_INITIALPRIORITIZATION<<"'."<<std::endl;
            initialprioritization = CN_DEFAULT_INITIALPRIORITIZATION;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_TIMELIMIT);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_TIMELIMIT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to "<<CNS_DEFAULT_TIMELIMIT<<"."<<std::endl;
        timelimit = CN_DEFAULT_TIMELIMIT;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>timelimit;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_RESCHEDULING);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_RESCHEDULING<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is to '"<<CNS_RE_NO<<"'."<<std::endl;
        rescheduling = CN_DEFAULT_RESCHEDULING;
    }
    else
    {
        value = element->GetText();
        if(value == CNS_RE_NO)
            rescheduling = CN_RE_NO;
        else if(value == CNS_RE_RULED)
            rescheduling = CN_RE_RULED;
        else if(value == CNS_RE_RANDOM)
            rescheduling = CN_RE_RANDOM;
        else
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_RESCHEDULING<<"' value. It's set to '"<<CNS_RE_NO<<"'."<<std::endl;
            rescheduling = CN_DEFAULT_RESCHEDULING;
        }
    }


    element = algorithm->FirstChildElement(CNS_TAG_PLANFORTURNS);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_PLANFORTURNS<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_PLANFORTURNS<<"'."<<std::endl;
        planforturns = CN_DEFAULT_PLANFORTURNS;
    }
    else
    {
        value = element->GetText();
        if(value == "true" || value == "1")
            planforturns = true;
        else if(value == "false" || value == "0")
            planforturns = false;
        else
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_PLANFORTURNS<<"' value. It's set to '"<<CNS_DEFAULT_PLANFORTURNS<<"'."<<std::endl;
            planforturns = CN_DEFAULT_PLANFORTURNS;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_ADDITIONALWAIT);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_ADDITIONALWAIT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CN_DEFAULT_ADDITIONALWAIT<<"'."<<std::endl;
        additionalwait = CN_DEFAULT_ADDITIONALWAIT;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>additionalwait;
        stream.clear();
        stream.str("");
        if(additionalwait < 0 || additionalwait > 100)
        {
            std::cout << "Warning! Wrong value of '"<<CNS_TAG_ADDITIONALWAIT<<"' element. It should belong to the interval [0,100]. Its value is set to '"<<CN_DEFAULT_ADDITIONALWAIT<<"'."<<std::endl;
            additionalwait = CN_DEFAULT_ADDITIONALWAIT;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_CONNECTEDNESS);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_CONNECTEDNESS<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CN_DEFAULT_CONNECTEDNESS<<"'."<<std::endl;
        connectedness = CN_DEFAULT_CONNECTEDNESS;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>connectedness;
        stream.clear();
        stream.str("");
        if(connectedness < 2 || connectedness > 5)
        {
            std::cout << "Warning! Wrong value of '"<<CNS_TAG_CONNECTEDNESS<<"' element. Possible variants are 2, 3, 4 or 5 . Its value is set to '"<<CN_DEFAULT_CONNECTEDNESS<<"'."<<std::endl;
            connectedness = CN_DEFAULT_CONNECTEDNESS;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_ALGTYPE);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_ALGTYPE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_ALGTYPE<<"'."<<std::endl;
        algtype = CN_DEFAULT_ALGTYPE;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>algtype;
        stream.clear();
        stream.str("");
        if(algtype < 1 || algtype > 5)
        {
            std::cout << "Warning! Wrong value of '"<<CNS_TAG_ALGTYPE<<"' element. Possible variants are 1, 2, 3, 4, 5. Its value is set to '"<<CNS_DEFAULT_ALGTYPE<<"'."<<std::endl;
            algtype = CN_DEFAULT_ALGTYPE;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_INFLATEINTERVALS);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_INFLATEINTERVALS<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CN_DEFAULT_INFLATEINTERVALS<<"'."<<std::endl;
        inflatecollisionintervals = CN_DEFAULT_INFLATEINTERVALS;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>inflatecollisionintervals;
        stream.clear();
        stream.str("");
        if(inflatecollisionintervals < 0 || inflatecollisionintervals > 100)
        {
            std::cout << "Warning! Wrong value of '"<<CNS_TAG_INFLATEINTERVALS<<"' element. It should belong to the interval [0,100]. Its value is set to '"<<CN_DEFAULT_INFLATEINTERVALS<<"'."<<std::endl;
            inflatecollisionintervals = CN_DEFAULT_INFLATEINTERVALS;
        }
    }

    XMLElement *options = root->FirstChildElement(CNS_TAG_OPTIONS);
    if(!options)
    {
        std::cout << "Warning! No '"<<CNS_TAG_OPTIONS<<"' section found."<<std::endl;
        loglevel = CN_DEFAULT_LOGLVL;
    }
    else
    {
        element = options->FirstChildElement(CNS_TAG_LOGLVL);
        if (!element)
        {
            std::cout << "Warning! No '"<<CNS_TAG_LOGLVL<<"' element found inside '"<<CNS_TAG_OPTIONS<<"' section. Its value is set to '"<<CNS_DEFAULT_LOGLVL<<"'."<<std::endl;
            loglevel = CN_DEFAULT_LOGLVL;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>loglevel;
            stream.clear();
            stream.str("");
        }
        if(loglevel != CN_LOGLVL_NO && loglevel != CN_LOGLVL_NORM && loglevel != CN_LOGLVL_ALL)
        {
            std::cout << "Warning! Wrong value of '"<<CNS_TAG_LOGLVL<<"' element found inside '"<<CNS_TAG_OPTIONS<<"' section. Its value is set to '"<<CNS_DEFAULT_LOGLVL<<"'."<<std::endl;
            loglevel = CN_DEFAULT_LOGLVL;
        }
        element = options->FirstChildElement(CNS_TAG_LOGPATH);
        if(element->GetText() != nullptr)
            logpath = element->GetText();
        element = options->FirstChildElement(CNS_TAG_LOGFILENAME);
        if(element->GetText() != nullptr)
            logfilename = element->GetText();
    }

    element = algorithm->FirstChildElement(CNS_TAG_FIXEDLOOKAHEADLIMIT);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_FIXEDLOOKAHEADLIMIT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_FIXEDLOOKAHEADLIMIT<<"'."<<std::endl;
        fixedlookahead = CNS_DEFAULT_FIXEDLOOKAHEADLIMIT;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>fixedlookahead;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_DYNMODE);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_DYNMODE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_DYNMODE<<"'."<<std::endl;
        dynmode = CNS_DEFAULT_DYNMODE;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>dynmode;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_LEARNINGALGORITHM);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_LEARNINGALGORITHM<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_LEARNINGALGORITHM<<"'."<<std::endl;
        learningalgorithm = CNS_DEFAULT_LEARNINGALGORITHM;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>learningalgorithm;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_DECISIONALGORITHM);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_DECISIONALGORITHM<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_DECISIONALGORITHM<<"'."<<std::endl;
        decisionalgorithm = CNS_DEFAULT_DECISIONALGORITHM;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>decisionalgorithm;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_EXPANSIONALGORITHM);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_EXPANSIONALGORITHM<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_EXPANSIONALGORITHM<<"'."<<std::endl;
        expansionalgorithm = CNS_DEFAULT_EXPANSIONALGORITHM;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>expansionalgorithm;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_MAXNUMOFINTEVALS);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_MAXNUMOFINTEVALS<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_MAXNUMOFINTEVALS<<"'."<<std::endl;
        maxNumOfIntervalsPerMove = CNS_DEFAULT_MAXNUMOFINTEVALS;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>maxNumOfIntervalsPerMove;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_UNITWAITDURATION);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_UNITWAITDURATION<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_UNITWAITDURATION<<"'."<<std::endl;
        unitWaitDuration = CNS_DEFAULT_UNITWAITDURATION;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>unitWaitDuration;
        stream.clear();
        stream.str("");
        if(unitWaitDuration < 0 || unitWaitDuration > 100)
        {
            std::cout << "Warning! Wrong value of '"<<CNS_TAG_UNITWAITDURATION<<"' element. It should belong to the interval [0,100]. Its value is set to '"<<CNS_DEFAULT_UNITWAITDURATION<<"'."<<std::endl;
            unitWaitDuration = CNS_DEFAULT_UNITWAITDURATION;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_IS_UNITWAITREPRESENTATION);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_IS_UNITWAITREPRESENTATION<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set '"<<CNS_DEFAULT_IS_UNITWAITREPRESENTATION<<"'."<<std::endl;
        isUnitWaitRepresentation = CNS_DEFAULT_IS_UNITWAITREPRESENTATION;
    }
    else
    {
        value = element->GetText();
        if(value == "true" || value == "1")
            isUnitWaitRepresentation = true;
        else if(value == "false" || value == "0")
            isUnitWaitRepresentation = false;
        else
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_IS_UNITWAITREPRESENTATION<<"' value. It's set to '"<<CNS_DEFAULT_IS_UNITWAITREPRESENTATION<<"'."<<std::endl;
            isUnitWaitRepresentation = CNS_DEFAULT_IS_UNITWAITREPRESENTATION;
        }
    }
    return true;
}
