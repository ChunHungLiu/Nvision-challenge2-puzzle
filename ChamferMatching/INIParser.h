#pragma once

#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <map>

#include <cctype>
#include <cstring>

#ifdef __INIPARSE_REPORT___
#define BUG_REPORT__(_errormsg) std::cout << _errormsg << std::endl
#else
#define BUG_REPORT__(_errormsg)
#endif

namespace ending{
	class INIparser{
	private:
		int _errorline = 0;
		std::string _errormsg = "";
		std::map<std::string, std::string> _value;

		char* lspace(char *p){
			while (*p && isspace(*p))p++;
			return p;
		}
		char* rspace(char *p, char *e){
			while (p > e && isspace(*p)){
				*p = '\0';
				p--;
			}
			return p;
		}

		char* find_char(char *p, char c){
			while (*p && *p != c)p++;
			return p;
		}

		std::string generateKey(const std::string &section, const std::string &name){
			std::string key = section + std::string(":") + name;
			std::transform(key.begin(), key.end(), key.begin(), tolower);
			return key;
		}

		void push_key(std::string &section, std::string &name, std::string &value){
			std::string key = generateKey(section, name);
			_value[key] = value;
		}

	public:

		INIparser(std::istream &in){
			parse(in);
		}

		int parse(std::istream &in){
			if (in.bad()){
				_errorline = 0;
				_errormsg = "No input stream...";
				BUG_REPORT__(_errormsg);
				return -1;
			}

			int linenum = 1;
			std::string section;
			std::string name;
			std::string value;

			while (!in.eof()){
				std::string line;
				std::getline(in, line, '\n');

				char *cline = new char[line.size() + 1];
				strcpy(cline, line.c_str());

				char *s = lspace(cline);
				char *e;

				if (*s == ';'){
					continue;

				}
				else if (*s == '['){

					e = find_char(s, ']');
					if (*e == ']'){

						*e = '\0';
						section = std::string(++s);

					}
					else{
						_errorline = linenum;
						_errormsg = "Illegal section in line " + std::to_string(_errorline);
						BUG_REPORT__(_errormsg);
					}

				}
				else if (*s){
					if (section == ""){
						_errorline = linenum;
						_errormsg = "No section in line " + std::to_string(_errorline);
						BUG_REPORT__(_errormsg);
					}
					else{

						e = find_char(s, '=');
						if (*e == '='){
							*e = '\0';
							rspace(s, e);
							name = std::string(s);
							s = e;
							s = lspace(++s);
							e = find_char(s, '\0');
							rspace(s, e);
							value = std::string(s);
							push_key(section, name, value);
						}
					}
				}

				linenum++;
				delete[] cline;
			}
			return 0;
		}

		bool error(){
			return _errorline != 0;
		}

		int errorline(){
			return _errorline;
		}

		std::string errormsg(){
			return _errormsg;
		}

		std::string getString(const std::string &section, const std::string &name, const std::string default_value = ""){
			std::string key = generateKey(section, name);
			std::map<std::string, std::string>::iterator it = _value.find(key);
			if (it == _value.end())return default_value;
			else return it->second;
		}

		int getInt(const std::string &section, const std::string &name, const int default_value = 0){
			std::string val = getString(section, name);
			return (val != "") ? stoi(val) : default_value;
		}

		double getDouble(const std::string &section, const std::string &name, const double default_value = 0){
			std::string val = getString(section, name);
			return (val != "") ? stod(val) : default_value;
		}
	};
}


