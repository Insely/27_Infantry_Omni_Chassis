#include "string.h"
#include "ui_interface.h"
#include "ui_default.h"
#include "ui_helm.h"

#define TOTAL_FIGURE 0
#define TOTAL_STRING 0


#ifndef MANUAL_DIRTY
#endif


void ui_init_default() {
    uint32_t idx = 0;

    SCAN_AND_SEND();

}

void ui_update_default() {
#ifndef MANUAL_DIRTY
#endif
    SCAN_AND_SEND();
}
