//
//  if_iwm_fw.h
//  AppleIntelIWM
//
//  Created by Felix McCuaig on 4/11/2019.
//  Copyright © 2019 Felix McCuaig. All rights reserved.
//

#ifndef if_iwm_fw_h
#define if_iwm_fw_h

/*
 * Block paging calculations
 */
#define IWM_PAGE_2_EXP_SIZE 12 /* 4K == 2^12 */
#define IWM_FW_PAGING_SIZE (1 << IWM_PAGE_2_EXP_SIZE) /* page size is 4KB */
#define IWM_PAGE_PER_GROUP_2_EXP_SIZE 3
/* 8 pages per group */
#define IWM_NUM_OF_PAGE_PER_GROUP (1 << IWM_PAGE_PER_GROUP_2_EXP_SIZE)
/* don't change, support only 32KB size */
#define IWM_PAGING_BLOCK_SIZE (IWM_NUM_OF_PAGE_PER_GROUP * IWM_FW_PAGING_SIZE)
/* 32K == 2^15 */
#define IWM_BLOCK_2_EXP_SIZE (IWM_PAGE_2_EXP_SIZE + IWM_PAGE_PER_GROUP_2_EXP_SIZE)

/*
 * Image paging calculations
 */
#define IWM_BLOCK_PER_IMAGE_2_EXP_SIZE 5
/* 2^5 == 32 blocks per image */
#define IWM_NUM_OF_BLOCK_PER_IMAGE (1 << IWM_BLOCK_PER_IMAGE_2_EXP_SIZE)
/* maximum image size 1024KB */
#define IWM_MAX_PAGING_IMAGE_SIZE (IWM_NUM_OF_BLOCK_PER_IMAGE * IWM_PAGING_BLOCK_SIZE)

/* Virtual address signature */
#define IWM_PAGING_ADDR_SIG 0xAA000000

#define IWM_PAGING_CMD_IS_SECURED (1 << 9)
#define IWM_PAGING_CMD_IS_ENABLED (1 << 8)
#define IWM_PAGING_CMD_NUM_OF_PAGES_IN_LAST_GRP_POS 0
#define IWM_PAGING_TLV_SECURE_MASK 1

#endif /* if_iwm_fw_h */
