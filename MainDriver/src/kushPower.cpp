#include <unistd.h>
#include <iostream>
#include "Log.h"
int main(void) {
	std::cout << "I am alive" << std::endl;
	char filename[] = "/tmp/mytemp.XXXXXX";
	int fd = mkstemp(filename);

	if (fd == -1) {
		std::cout << "PAIN" << std::endl;
		return 1;
	}
	write(fd,"abc",4);
	close(fd);
	unlink(filename);
	return 0;
}