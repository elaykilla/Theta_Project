/*
* @author: ELay Maiga
* 
*/


#ifndef BOOST_HEADERS_H
#define BOOST_HEADERS_H
//Boost Includes
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
//#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/filesystem.hpp>
#include <time.h>
#include <boost/thread.hpp>


#include "standard_headers.hpp" 

namespace logging = boost::log;
/**
* Initiates a log file
*/
//void InitLog(string filename){
//	ostringstream name;
//	name << filename << ".log";
//	
//	//logging::add_file_log(name.str());
////	boost::log::add_file_log (
////    boost::log::keywords::file_name = "MyApp_%3N.log",
////    boost::log::keywords::rotation_size = 1 * 1024 * 1024,
////    boost::log::keywords::max_size = 20 * 1024 * 1024,
////    boost::log::keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0, 0, 0),
////    boost::log::keywords::format = "[%TimeStamp%]: %Message%",
////    boost::log::keywords::auto_flush = true
////  );	
//	
//	
//	logging::core::get()->set_filter
//	(
//        logging::trivial::severity >= logging::trivial::info
//	);

//}

#endif
