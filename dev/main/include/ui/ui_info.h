#ifndef UI_INFO_H
#define UI_INFO_H

#include <stdbool.h>

/**
 * @brief Get the number of info pages for the current main page
 * @return Number of info pages available
 */
int ui_info_get_page_count(void);

/**
 * @brief Draw an info page for the current main page
 * @param info_page_num Which info page to draw (0-based)
 */
void ui_draw_info_page(int info_page_num);

#endif // UI_INFO_H
