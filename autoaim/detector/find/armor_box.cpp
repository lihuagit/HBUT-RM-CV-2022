//
// Created by xinyang on 19-7-13.
//

#include "../detector.h"
#include <log.h>

// 装甲板的优先级比较
bool ArmorBox::operator<(const ArmorBox &box) const {
    if (tag_id != box.tag_id) {
        if (box_color == BOX_BLUE) {
            return prior_blue[id2name[tag_id]] < prior_blue[id2name[box.tag_id]];
        } else {
            return prior_red[id2name[tag_id]] < prior_red[id2name[box.tag_id]];
        }
    } else {
        auto d1 = (rect.x - IMAGE_CENTER_X) * (rect.x - IMAGE_CENTER_X)
                  + (rect.y - IMAGE_CENTER_Y) * (rect.y - IMAGE_CENTER_Y);
        auto d2 = (box.rect.x - IMAGE_CENTER_X) * (box.rect.x - IMAGE_CENTER_X)
                  + (box.rect.y - IMAGE_CENTER_Y) * (box.rect.y - IMAGE_CENTER_Y);
        return d1 < d2;
    }
}