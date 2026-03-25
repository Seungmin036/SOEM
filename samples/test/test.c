/*
 * This software is dual-licensed under GPLv3 and a commercial
 * license. See the file LICENSE.md distributed with this software for
 * full license information.
 */

#include <stdio.h>
#include <stdlib.h>

#include "soem/soem.h"

static ecx_contextt ctx;

static const char *state_to_string(uint16 state)
{
   state &= (uint16)~EC_STATE_ACK;

   switch (state)
   {
   case EC_STATE_INIT:
      return "INIT";
   case EC_STATE_PRE_OP:
      return "PRE_OP";
   case EC_STATE_BOOT:
      return "BOOT";
   case EC_STATE_SAFE_OP:
      return "SAFE_OP";
   case EC_STATE_OPERATIONAL:
      return "OP";
   default:
      return "UNKNOWN";
   }
}

static int sdo_read_u8(uint16 slave, uint16 index, uint8 subidx, uint8 *value)
{
   int size = sizeof(*value);
   int wkc = ecx_SDOread(&ctx, slave, index, subidx, FALSE, &size, value, EC_TIMEOUTRXM);
   if(wkc <= 0)
   {
      printf("SDO read failed for 0x%04X:%02X on slave %u, wkc=%d\n", index, subidx, slave, wkc);
      return FALSE;
   }
   else if (size != (int)sizeof(*value))
   {
      printf("SDO read size mismatch for 0x%04X:%02X on slave %u, expected=%zu got=%d\n",
             index, subidx, slave, sizeof(*value), size);
      return FALSE;
   }
   // printf("wkc=%d, size=%d,  (int)sizeof(*value)=%d\n", wkc, size,  (int)sizeof(*value));
   return (wkc > 0) && (size == (int)sizeof(*value));
}

static int sdo_read_u16(uint16 slave, uint16 index, uint8 subidx, uint16 *value)
{
   int size = sizeof(*value);
   int wkc = ecx_SDOread(&ctx, slave, index, subidx, FALSE, &size, value, EC_TIMEOUTRXM);
   if(wkc <= 0)
   {
      printf("SDO read failed for 0x%04X:%02X on slave %u, wkc=%d\n", index, subidx, slave, wkc);
      return FALSE;
   }
   else if (size != (int)sizeof(*value))
   {
      printf("SDO read size mismatch for 0x%04X:%02X on slave %u, expected=%zu got=%d\n", index, subidx, slave, sizeof(*value), size);
      return FALSE;
   }
   return (wkc > 0) && (size == (int)sizeof(*value));
}

static int sdo_read_u32(uint16 slave, uint16 index, uint8 subidx, uint32 *value)
{
   int size = sizeof(*value);
   int wkc = ecx_SDOread(&ctx, slave, index, subidx, FALSE, &size, value, EC_TIMEOUTRXM);
   if(wkc <= 0)
   {
      printf("SDO read failed for 0x%04X:%02X on slave %u, wkc=%d\n", index, subidx, slave, wkc);
      return FALSE;
   }
   else if (size != (int)sizeof(*value))
   {
      printf("SDO read size mismatch for 0x%04X:%02X on slave %u, expected=%zu got=%d\n",
             index, subidx, slave, sizeof(*value), size);
      return FALSE;
   }
   return (wkc > 0) && (size == (int)sizeof(*value));
}

static int sdo_write_u16(uint16 slave, uint16 index, uint8 subidx, uint16 value)
{
   int size = sizeof(value);
   int wkc = ecx_SDOwrite(&ctx, slave, index, subidx, FALSE, size, &value, EC_TIMEOUTRXM);
   return (wkc > 0);
}

static void drain_errors(void)
{
   while (ctx.ecaterror)
   {
      printf("  %s", ecx_elist2string(&ctx));
   }
}

static void print_sdo_failure(uint16 slave, uint16 index, uint8 subidx)
{
   uint16 state = ctx.slavelist[slave].state;

   printf("0x%04X:%02X read failed on slave %u (state=0x%02X %s)\n",
          index, subidx, slave, state, state_to_string(state));
   drain_errors();
}

static void dump_assign_object(uint16 slave, uint16 index)
{
   uint8 count = 0;

   printf("Reading 0x%04X:00\n", index);

   if (!sdo_read_u8(slave, index, 0x00, &count))
   {
      print_sdo_failure(slave, index, 0x00);
      return;
   }

   printf("0x%04X assignment count: %u\n", index, count);
   for (uint8 sub = 1; sub <= count; ++sub)
   {
      uint16 pdo_index = 0;

      if (!sdo_read_u16(slave, index, sub, &pdo_index))
      {
         print_sdo_failure(slave, index, sub);
         continue;
      }

      printf("  0x%04X:%02X = 0x%04X\n", index, sub, pdo_index);
   }
}

static void dump_map_object(uint16 slave, uint16 index)
{
   uint8 count = 0;

   printf("Reading 0x%04X:00\n", index);

   if (!sdo_read_u8(slave, index, 0x00, &count))
   {
      print_sdo_failure(slave, index, 0x00);
      return;
   }

   printf("0x%04X mapped entry count: %u\n", index, count);
   for (uint8 sub = 1; sub <= count; ++sub)
   {
      uint32 map = 0;
      uint16 obj_index = 0;
      uint8 obj_subidx = 0;
      uint8 bitlen = 0;

      if (!sdo_read_u32(slave, index, sub, &map))
      {
         print_sdo_failure(slave, index, sub);
         continue;
      }

      obj_index = (uint16)((map >> 16) & 0xffffU);
      obj_subidx = (uint8)((map >> 8) & 0xffU);
      bitlen = (uint8)(map & 0xffU);

      printf("  0x%04X:%02X = 0x%08X -> 0x%04X:%02X (%u bits)\n",
             index, sub, map, obj_index, obj_subidx, bitlen);
   }
}

static void dump_fixed_objects(uint16 slave)
{
   static const uint16 assign_indices[] = {0x1C12, 0x1C13};
   static const uint16 rxpdo_maps[] = {0x1703};
   static const uint16 txpdo_maps[] = {0x1B03};
   size_t i;

   printf("[PDO assignment]\n");
   for (i = 0; i < sizeof(assign_indices) / sizeof(assign_indices[0]); ++i)
   {
      dump_assign_object(slave, assign_indices[i]);
   }

   printf("\n[RxPDO mapping]\n");
   for (i = 0; i < sizeof(rxpdo_maps) / sizeof(rxpdo_maps[0]); ++i)
   {
      dump_map_object(slave, rxpdo_maps[i]);
   }

   printf("\n[TxPDO mapping]\n");
   for (i = 0; i < sizeof(txpdo_maps) / sizeof(txpdo_maps[0]); ++i)
   {
      dump_map_object(slave, txpdo_maps[i]);
   }
}

static uint8 IOmap[4096];

int main(int argc, char *argv[])
{
   const char *ifname = NULL;
   uint16 slave = 1;

   if ((argc < 2) || (argc > 3))
   {
      printf("Usage: %s <ifname> [slave]\n", argv[0]);
      return EXIT_FAILURE;
   }

   ifname = argv[1];
   if (argc == 3)
   {
      slave = (uint16)strtoul(argv[2], NULL, 0);
   }

   if (!ecx_init(&ctx, ifname))
   {
      printf("Failed to open interface %s\n", ifname);
      return EXIT_FAILURE;
   }

   if (ecx_config_init(&ctx) <= 0)
   {
      printf("No EtherCAT slaves found\n");
      ecx_close(&ctx);
      return EXIT_FAILURE;
   }

   if ((slave == 0U) || (slave > ctx.slavecount))
   {
      printf("Invalid slave index %u, detected slaves=%d\n", slave, ctx.slavecount);
      ecx_close(&ctx);
      return EXIT_FAILURE;
   }

   printf("Initial master state : 0x%02X %s\n",
          ctx.slavelist[0].state, state_to_string(ctx.slavelist[0].state));
   printf("Initial slave state  : 0x%02X %s\n",
          ctx.slavelist[slave].state, state_to_string(ctx.slavelist[slave].state));

   /* 1) 전역 PRE_OP 요청 */
   ctx.slavelist[0].state = EC_STATE_PRE_OP;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

   /* 2) 타겟 슬레이브 상태 명시적 확인 (필요 시 직접 요청) */
   if (ctx.slavelist[slave].state != EC_STATE_PRE_OP)
   {
      printf("Slave %u is not PRE_OP yet, requesting directly...\n", slave);
      ctx.slavelist[slave].state = EC_STATE_PRE_OP;
      ecx_writestate(&ctx, slave);
      ecx_statecheck(&ctx, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
   }

   printf("After PRE_OP request master state : 0x%02X %s\n",
          ctx.slavelist[0].state, state_to_string(ctx.slavelist[0].state));
   printf("After PRE_OP request slave state  : 0x%02X %s\n\n",
          ctx.slavelist[slave].state, state_to_string(ctx.slavelist[slave].state));

   if (ctx.slavelist[slave].state != EC_STATE_PRE_OP)
   {
      printf("ERROR: slave %u failed to reach PRE_OP (0x%02X %s)\n",
             slave, ctx.slavelist[slave].state, state_to_string(ctx.slavelist[slave].state));
      ecx_close(&ctx);
      return EXIT_FAILURE;
   }
   dump_fixed_objects(slave);

   printf("Starting config map/dc + SAFE_OP/OP state transitions\n");
   if (!ecx_config_map_group(&ctx, IOmap, 0))
   {
      printf("ecx_config_map_group failed\n");
      ecx_close(&ctx);
      return EXIT_FAILURE;
   }

   // ecx_configdc(&ctx);
   /*MASTER state check*/
   ctx.slavelist[0].state = EC_STATE_SAFE_OP;
   ctx.slavelist[slave].state = EC_STATE_SAFE_OP;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
   /*SLAVE state check*/
   ecx_writestate(&ctx, slave);
   ecx_statecheck(&ctx, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

   printf("After SAFE_OP request master state : 0x%02X %s\n",
          ctx.slavelist[0].state, state_to_string(ctx.slavelist[0].state));
   printf("After SAFE_OP request slave state  : 0x%02X %s\n",
          ctx.slavelist[slave].state, state_to_string(ctx.slavelist[slave].state));
   printf("SAFE_OP OK\n");

   /* ================= Send process data before requesting OP ================== */
   printf("Sending process data before OP request...\n");
   for (int i = 0; i < 100; i++) {
      ecx_send_processdata(&ctx);
      ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      osal_usleep(1000);  // 1 ms
   }
   ctx.slavelist[0].state = EC_STATE_OPERATIONAL;

   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
   ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
   printf("After OP request master state : 0x%02X %s\n",
          ctx.slavelist[0].state, state_to_string(ctx.slavelist[0].state));
   printf("After OP request slave state  : 0x%02X %s\n\n",
          ctx.slavelist[slave].state, state_to_string(ctx.slavelist[slave].state));

   
   ecx_readstate(&ctx);
   printf("After OP request master state : 0x%02X %s\n",
         ctx.slavelist[0].state, state_to_string(ctx.slavelist[0].state));
   printf("After OP request slave state  : 0x%02X %s\n",
         ctx.slavelist[slave].state, state_to_string(ctx.slavelist[slave].state));
   printf("AL status code : 0x%04X\n", ctx.slavelist[slave].ALstatuscode);
   printf("AL status text : %s\n", ec_ALstatuscode2string(ctx.slavelist[slave].ALstatuscode));
   ecx_close(&ctx);
   return EXIT_SUCCESS;
}
