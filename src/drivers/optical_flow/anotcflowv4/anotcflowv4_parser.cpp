#include "anotcflowv4_parser.h"

bool anotc_check(char *data_buf) {
    char sumcheck = 0;
    char addcheck = 0;

    for (int i = 0; i < (data_buf[3] + 4); i++) {
        sumcheck += data_buf[i];
        addcheck += sumcheck;
    }
    if (sumcheck == data_buf[data_buf[3] + 4] && addcheck == data_buf[data_buf[3] + 5])
        return true;
    else
        return false;
}

int anotcflowv4_parse_of_origin(char c, char *parserbuf, unsigned *parserbuf_index,
                                ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN *state,
                                ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN_STRUCT *result) {
    int ret = -1;
    switch (*state) {
        case ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE1_HEAD:
            if (c == ANOTCFLOWV4_PROTOCOL_HEAD) {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE2_D_ADDR;
                parserbuf[*parserbuf_index] = c;
                (*parserbuf_index)++;
            } else {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE1_HEAD;
                *parserbuf_index = 0;
            }
            break;

        case ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE2_D_ADDR:
            if (c == ANOTCFLOWV4_PROTOCOL_D_ADDR) {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE3_FUNCTION_ID;
                parserbuf[*parserbuf_index] = c;
                (*parserbuf_index)++;
            } else {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE1_HEAD;
                *parserbuf_index = 0;
            }
            break;
        case ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE3_FUNCTION_ID:
            if (c == ANOTCFLOWV4_PROTOCOL_FUNCTION_ID_OF) {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE4_LEN;
                parserbuf[*parserbuf_index] = c;
                (*parserbuf_index)++;
            } else {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE1_HEAD;
                *parserbuf_index = 0;
            }
            break;
        case ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE4_LEN:
            if (c == ANOTCFLOWV4_PROTOCOL_LEN_OF_ORIGIN) {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE5_MODE_OF_ORIGIN;
                parserbuf[*parserbuf_index] = c;
                (*parserbuf_index)++;
            } else {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE1_HEAD;
                *parserbuf_index = 0;
            }
            break;
        case ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE5_MODE_OF_ORIGIN:
            if (c == ANOTCFLOWV4_PROTOCOL_MODE) {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE6_VALID_STATE_OF_ORIGIN;
                parserbuf[*parserbuf_index] = c;
                (*parserbuf_index)++;
            } else {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE1_HEAD;
                *parserbuf_index = 0;
            }
            break;
        case ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE6_VALID_STATE_OF_ORIGIN:
            if (c == ANOTCFLOWV4_PROTOCOL_VALID_STATE) {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE7_DX_0_OF_ORIGIN;
                parserbuf[*parserbuf_index] = c;
                (*parserbuf_index)++;
            } else {
                *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE1_HEAD;
                *parserbuf_index = 0;
            }
            break;
        case ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE7_DX_0_OF_ORIGIN:
            *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE8_DY_0_OF_ORIGIN;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;
            break;
        case ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE8_DY_0_OF_ORIGIN:
            *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE9_QUALITY_OF_ORIGIN;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;
            break;
        case ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE9_QUALITY_OF_ORIGIN:
            *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE10_SC_VALUE_OF_ORIGIN;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;
            break;
        case ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE10_SC_VALUE_OF_ORIGIN:
            *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE11_AC_VALUE_OF_ORIGIN;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;
            break;
        case ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE11_AC_VALUE_OF_ORIGIN:

            *state = ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE1_HEAD;
            parserbuf[*parserbuf_index] = c;
            *parserbuf_index = 0;
            bool ret_check = anotc_check(parserbuf);
            if (ret_check == true) {
                result->dx_0 = parserbuf[static_cast<int>(ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE7_DX_0_OF_ORIGIN)];
                result->dy_0 = parserbuf[static_cast<int>(ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE8_DY_0_OF_ORIGIN)];
		result->quality = parserbuf[static_cast<int>(ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE9_QUALITY_OF_ORIGIN)];
                ret = 0;
            }
            break;
    }
    return ret;
}
