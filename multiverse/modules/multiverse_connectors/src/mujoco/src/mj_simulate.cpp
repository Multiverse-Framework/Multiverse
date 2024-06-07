// Copyright (c) 2023, Giang Hoang Nguyen - Institute for Artificial Intelligence, University Bremen

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "mj_simulate.h"

MjSimulate::~MjSimulate()
{
    mj_deleteData(d);
    mj_deleteModel(m);
}

void MjSimulate::init()
{
    // load and compile model
    char error[1000] = "Could not load binary model";

    m = mj_loadXML(scene_xml_path.string().c_str(), 0, error, 1000);

    if (!m)
    {
        mju_error("Load model error: %s", error);
    }

    // make data
    d = mj_makeData(m);

	mj_resetDataKeyframe(m, d, 0);
}

void MjSimulate::load_new_model_and_keep_old_data()
{
    // load and compile model
    char error[1000] = "Could not load binary model";

    mjModel *m_new = mj_loadXML(scene_xml_path.string().c_str(), 0, error, 1000);

    if (!m_new)
    {
        mju_error("Load model error: %s", error);
    }

    // make data
    mjData *d_new = mj_makeData(m_new);

	// mj_resetDataKeyframe(m_new, d_new, 0);

	d_new->time = d->time;

	int body_id = 0;
	while (true)
	{
		body_id++;
		const char *name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
		if (name == nullptr)
		{
			break;
		}

		int body_id_new = mj_name2id(m_new, mjtObj::mjOBJ_BODY, name);
		if (body_id_new == -1)
		{
			continue;
		}

		// Copy body position
		d_new->xpos[body_id_new] = d->xpos[body_id];
		d_new->xpos[body_id_new + 1] = d->xpos[body_id + 1];
		d_new->xpos[body_id_new + 2] = d->xpos[body_id + 2];

		// Copy body rotation
		d_new->xquat[body_id_new] = d->xquat[body_id];
		d_new->xquat[body_id_new + 1] = d->xquat[body_id + 1];
		d_new->xquat[body_id_new + 2] = d->xquat[body_id + 2];
		d_new->xquat[body_id_new + 3] = d->xquat[body_id + 3];

		// Copy body states
		for (int body_nr = 0; body_nr < 6; body_nr++)
		{
			d_new->xfrc_applied[body_id_new + body_nr] = d->xfrc_applied[body_id + body_nr];
		}

		// Copy joint states
		int jnt_num = m->body_jntnum[body_id];
		int jnt_num_new = m_new->body_jntnum[body_id_new];
		if (jnt_num == jnt_num_new)
		{
			int qpos_adr = m->jnt_qposadr[m->body_jntadr[body_id]];
			int qpos_adr_new = m_new->jnt_qposadr[m_new->body_jntadr[body_id_new]];
			for (int jnt_nr = 0; jnt_nr < jnt_num; jnt_nr++)
			{
				d_new->qpos[qpos_adr_new + jnt_nr] = d->qpos[qpos_adr + jnt_nr];
			}
		}
		else
		{
			printf("Old [%s] has %d joints != new [%s] has %d joints\n", name, jnt_num, name, jnt_num_new);
		}

		// Copy dof states
		int dof_num = m->body_dofnum[body_id];
		int dof_num_new = m_new->body_dofnum[body_id_new];
		if (dof_num == dof_num_new)
		{
			int dof_adr = m->jnt_dofadr[m->body_jntadr[body_id]];
			int dof_adr_new = m_new->jnt_dofadr[m_new->body_jntadr[body_id_new]];
			for (int dof_nr = 0; dof_nr < dof_num; dof_nr++)
			{
				d_new->qvel[dof_adr_new + dof_nr] = d->qvel[dof_adr + dof_nr];
				d_new->qacc_warmstart[dof_adr_new + dof_nr] = d->qacc_warmstart[dof_adr + dof_nr];
				d_new->qfrc_applied[dof_adr_new + dof_nr] = d->qfrc_applied[dof_adr + dof_nr];
				d_new->qacc[dof_adr_new + dof_nr] = d->qacc[dof_adr + dof_nr];
			}
		}
		else
		{
			printf("Old [%s] has %d dofs != new [%s] has %d dofs\n", name, dof_num, name, dof_num_new);
		}
	}

	// Copy activation states
	if (m->na != 0 || m_new->na != 0)
	{
		printf("Old model has %d activation states, new model has %d activation states, not supported, will be ignored...", m->na, m_new->na);
	}

	// Copy sensor data
	if (m->nsensordata != m_new->nsensordata)
	{
		printf("Old model has %d sensors, new model has %d sensors, not supported, will be ignored...", m->nsensordata, m_new->nsensordata);
	}
	else
	{
		mju_copy(d_new->sensordata, d->sensordata, m->nsensordata);
	}

	d = d_new;
	m = m_new;
}