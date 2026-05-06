#include <stdio.h>
#include "device/ecMaster.h"

ec_master_t *master = NULL;
ec_domain_t *domain = NULL;
ec_slave_config_t *sc = NULL;
uint8_t *pPdoList = NULL;
ec_master_state_t masterState;
ec_domain_state_t domainState;



int8_t ecMasterInit(SysData *sysData)
{
    fprintf(stderr, "Requesting EtherCAT master ... ");

    master = ecrt_request_master(sysData->ecMasterIndex);
    if (!master)
    {
        fprintf(stderr, "error: ecrt_request_master(...)\n");
        return -1;
    }
    else fprintf(stderr, "ok\n");
    

    /* 2. Create PDO Domain */
    fprintf(stderr, "Create process data domain ... ");
    domain = ecrt_master_create_domain(master);
    if (!domain)
    {
        fprintf(stderr, "error: ecrt_master_create_domain(...)\n");
        return -1;
    }
    else fprintf(stderr, "ok\n");

    ecrt_master_state(master, &masterState);
    ecrt_domain_state(domain, &domainState);

    sysData->ecMasterState = &masterState ;
    sysData->ecDomainState = &domainState ;

    return 0;
}


int8_t ecSetSlavePdo(uint16_t alias, uint16_t slavePos, uint32_t vid, uint32_t pid, const ec_sync_info_t syncs[])
{
    // ec_slave_config_t *sc;
    fprintf(stderr, "Slave %d: Get slave configuration ... ", slavePos);
    sc = ecrt_master_slave_config(master, alias, slavePos, vid, pid);
    if (!sc)
    {
        fprintf(stderr, "error: ecrt_master_slave_config(...)\n");
        return -1;
    }
    fprintf(stderr, "ok\n");

    /* 4. Configure Slave PDOs */
    if (syncs != NULL)
    {
        fprintf(stderr, "Slave %d: Configure PDOs ... ", slavePos);
        if (ecrt_slave_config_pdos(sc, EC_END, syncs))
        {
            fprintf(stderr, "error: ecrt_slave_config_pdos(...)\n");
            return -1;
        }
        fprintf(stderr, "ok\n");
    }

    return 0;
}

int8_t ecRegDomainPdoEntry(uint16_t index, uint8_t subindex, int *offset)
{

    int rtn;

    /* 5. Register PDO Entry list - Configure process data for a domain */
    fprintf(stderr, "Slave Register PDO Entry to Domain ... ");
    rtn = ecrt_slave_config_reg_pdo_entry(sc, index, subindex, domain, NULL);
    if (rtn < 0)
    {
        fprintf(stderr, "error: ecrt_slave_config_reg_pdo_entry(...)\n");
        return -1;
    }

    if (offset) *offset = rtn;

    fprintf(stderr, "ok\n");
    
    return 0;
}

int8_t ecStart(SysData *sysData)
{
    fprintf(stderr, "Activating EtherCAT Master ... ");
    if (ecrt_master_activate(master))
    {
        fprintf(stderr, "error: ecrt_master_activate(...)\n");
        return -1;
    }
    fprintf(stderr, "ok\n");

    /* 7. Get the domain data pointer */
    fprintf(stderr, "Get the domain process data pointer ... ");
    sysData->pPdoList = ecrt_domain_data(domain);
    if (!sysData->pPdoList)
    {
        fprintf(stderr, "error: ecrt_domain_data(...)\n");
        return -1;
    }
    fprintf(stderr, "ok\n");

    return 0;
}

int8_t ecMasterM2S()
{
    ecrt_domain_queue(domain);
    ecrt_master_send(master);

    return 0;
}

int8_t ecMasterS2M()
{
    ecrt_master_receive(master);
    // Determines the states of the domain's datagrams.
    ecrt_domain_process(domain);

    // Get DomainState & MasterState
    ecrt_master_state(master, &masterState);
    ecrt_domain_state(domain, &domainState);

    return 0;
}