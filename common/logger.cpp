#include <logger.h>

	Logger::Logger() { 
			logFile = 0; 
		strcpy(data_directory,"/home/arvind/");	
		strcpy(full_name,"");
		createDirectory();
	}


	Logger::Logger(char *prefix) {
		logFile = 0; 
		strcpy(data_directory,"/home/arvind/");	
		strcpy(full_name,"");
		createDirectory();
		openRoboducklogfile(prefix);
	}

	void Logger::createDirectory()
	{
 		sprintf(full_name, "%s%%Y%%m%%d/", data_directory);
  		time(&t);
  		tm = localtime(&t);
  		len = strftime(dir_name, sizeof(dir_name) - 1, full_name, tm);
  		if(-1 == mkdir(dir_name, 0700))
		{
			if(errno != EEXIST)
			{
				perror("roboduck unable to create directory");
			}
  		}
	}
	bool Logger::openRoboducklogfile(char *prefix)
	{
		char file_name[100];
		gettimeofday(&tv,NULL);
		strncpy(full_name, dir_name, sizeof(full_name)-1);
		sprintf(file_name,"%s%ld",prefix,tv.tv_sec);
		strcat(full_name,file_name);
		if ((logFile = fopen(full_name, "wb")) == NULL)
		{
			fprintf(stderr, "cannot open %s for binary file", full_name);
			perror("and the reason is");
	                return false;
		}
		else return true;
	}

	bool Logger::closeLogFile()
	{
		if(logFile!=0) {
			fclose(logFile);
		}	
	}
