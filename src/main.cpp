#include "controller.h"

int
main (int argc,
      char *argv[])
{
  boost::shared_ptr<Controller> control (new Controller);
  control->init (argc, argv);
}
