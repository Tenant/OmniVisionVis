#include "velointrinsic.h"

VeloXMLIntrinsicParams::VeloXMLIntrinsicParams() {
    linenum = 0;
}

bool VeloXMLIntrinsicParams::loadCalib(std::string filename) {
    linenum = 64;
    for (size_t i = 0; i != linenum; ++i) idx.push_back(i);
	tinyxml2::TinyXMLDocument file;
	file.LoadFile(filename.c_str());

	XMLElement* root = file.RootElement();
	XMLElement* p = root->FirstChildElement()->FirstChildElement();

    while(p!=NULL) {
		std::string nodeName = p->Name();

        if (nodeName == "distLSB_") {
            std::string text =  p->GetText();
            unit = std::stod(text);
        }
        if (nodeName == "position_") {
			XMLElement* xyz = p->FirstChildElement();
			xyz = xyz->FirstChildElement("item");
			position_x = std::stod(xyz->GetText());
			xyz = xyz->NextSiblingElement("item");
			position_y = std::stod(xyz->GetText());
			xyz = xyz->NextSiblingElement("item");
			position_z = std::stod(xyz->GetText());
        }
        if (nodeName == "orientation_") {
			XMLElement* rpy = p->FirstChildElement();
			rpy = rpy->FirstChildElement("item");
			orientation_r = std::stod(rpy->GetText());
			rpy = rpy->NextSiblingElement("item");
			orientation_p = std::stod(rpy->GetText());
			rpy = rpy->NextSiblingElement("item");
			orientation_y = std::stod(rpy->GetText());
        }
//        if (nodeName == "colors_") {
//            QDomElement item = node.firstChildElement("item");
//            while (!item.isNull()) {
//                QDomElement rgb = item.firstChildElement();
//                rgb = rgb.firstChildElement("item");
//                double r =  rgb.text().toDouble();
//                rgb = rgb.nextSiblingElement("item");
//                double g =  rgb.text().toDouble();
//                rgb = rgb.nextSiblingElement("item");
//                double b =  rgb.text().toDouble();
//                colors_r << r;
//                colors_g << g;
//                colors_b << b;
//                item = item.nextSiblingElement("item");
//            }
//        }
        if (nodeName == "enabled_") {
            XMLElement* item = p->FirstChildElement("item");
            while (item!=NULL) {
                enabled.push_back(item->GetText() == "1");
				item = item->NextSiblingElement("item");
            }
        }
        if (nodeName == "intensity_") {
			XMLElement* item = p->FirstChildElement("item");
			while (item != NULL) {
				intensity.push_back(item->GetText() == "1");
				item = item->NextSiblingElement("item");
            }
        }
        if (nodeName == "minIntensity_") {
			XMLElement* item = p->FirstChildElement("item");
            while (item!=NULL) {
				minIntensity.push_back(std::stod(item->GetText()));
				item = item->NextSiblingElement("item");
            }
        }
        if (nodeName == "maxIntensity_") {
			XMLElement* item = p->FirstChildElement("item");
            while (item!=NULL) {
				maxIntensity.push_back(std::stod(item->GetText()));
				item = item->NextSiblingElement("item");
            }
        }
        if (nodeName == "points_") {
			XMLElement* item = p->FirstChildElement("item");
            while (item!=NULL) {
				XMLElement* px = item->FirstChildElement(); 
                rotCorrection.push_back( std::stod(px->FirstChildElement("rotCorrection_")->GetText())/ 180.0 * _M_PI);
				vertCorrection.push_back(std::stod(px->FirstChildElement("vertCorrection_")->GetText()) / 180.0 * _M_PI);
				distCorrection.push_back(std::stod(px->FirstChildElement("distCorrection_")->GetText()));
				distCorrectionX.push_back(std::stod(px->FirstChildElement("distCorrectionX_")->GetText()));
				distCorrectionY.push_back(std::stod(px->FirstChildElement("distCorrectionY_")->GetText()));
				vertOffsetCorrection.push_back(std::stod(px->FirstChildElement("vertOffsetCorrection_")->GetText()));
				horizOffsetCorrection.push_back(std::stod(px->FirstChildElement("horizOffsetCorrection_")->GetText()));
				focalDistance.push_back(std::stod(px->FirstChildElement("focalDistance_")->GetText()));
				focalSlope.push_back(std::stod(px->FirstChildElement("focalSlope_")->GetText()));
				item = item->NextSiblingElement("item");
            }
        }
		p = p->NextSiblingElement();
    }
    minAngle = vertCorrection[0];
    maxAngle = vertCorrection[0];
    for (int i = 1; i < vertCorrection.size(); i++) {
        if (vertCorrection[i] > maxAngle) {
            maxAngle = vertCorrection[i];
        }
        if (vertCorrection[i] < minAngle) {
            minAngle = vertCorrection[i];
        }
    }

    linenum = 32;
    if (vertCorrection[32] > 0) {
        linenum = 64;
    }
    std::sort(idx.begin(), idx.begin() + linenum, [this](size_t i1, size_t i2) {
        return vertCorrection[i1] <  vertCorrection[i2];
    });
    return true;
}
