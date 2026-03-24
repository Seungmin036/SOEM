/*
 * This software is dual-licensed under GPLv3 and a commercial
 * license. See the file LICENSE.md distributed with this software for
 * full license information.
 */

#include <stdio.h>
#include <stdlib.h>

#include "soem/soem.h"

static ecx_contextt ctx;

static int sdo_read_u8(uint16 slave, uint16 index, uint8 subidx, uint8 *value)
{
   int size = sizeof(*value);
   int wkc = ecx_SDOread(&ctx, slave, index, subidx, FALSE, &size, value, EC_TIMEOUTRXM);
   return (wkc > 0) && (size == (int)sizeof(*value));
}

static int sdo_read_u16(uint16 slave, uint16 index, uint8 subidx, uint16 *value)
{
   int size = sizeof(*value);
   int wkc = ecx_SDOread(&ctx, slave, index, subidx, FALSE, &size, value, EC_TIMEOUTRXM);
   return (wkc > 0) && (size == (int)sizeof(*value));
}

static int sdo_read_u32(uint16 slave, uint16 index, uint8 subidx, uint32 *value)
{
   int size = sizeof(*value);
   int wkc = ecx_SDOread(&ctx, slave, index, subidx, FALSE, &size, value, EC_TIMEOUTRXM);
   return (wkc > 0) && (size == (int)sizeof(*value));
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
   printf("0x%04X:%02X read failed on slave %u\n", index, subidx, slave);
   drain_errors();
}

static void dump_assign_object(uint16 slave, uint16 index)
{
   uint8 count = 0;

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
   static const uint16 rxpdo_maps[] = {0x1600, 0x1601, 0x1602, 0x1603};
   static const uint16 txpdo_maps[] = {0x1A00, 0x1A01, 0x1A02, 0x1A03};
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

   ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

   printf("Target slave: %u\n", slave);
   printf("Name        : %s\n", ctx.slavelist[slave].name);
   printf("Vendor ID   : 0x%08X\n", ctx.slavelist[slave].eep_man);
   printf("Product Code: 0x%08X\n", ctx.slavelist[slave].eep_id);
   printf("Revision    : 0x%08X\n\n", ctx.slavelist[slave].eep_rev);

   dump_fixed_objects(slave);

   ecx_close(&ctx);
   return EXIT_SUCCESS;
}
